"""离线零偏验证与候选静止检测（不是 INS/ZUPT 更新）。
Offline bias validation and candidate stationary detection (not INS/ZUPT updates).
运行 / Run: python zupt_offline.py
仅分析 / Analysis only: python zupt_offline.py --no-gui
"""
import argparse
import bisect
import hashlib
import json
import math
import statistics as st
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parent
DEFAULT = ROOT / 'recordings/20261007_155937_947511_motion'
FIELDS = ('ax_mps2', 'ay_mps2', 'az_mps2', 'gx_radps', 'gy_radps', 'gz_radps')
G = 9.80665


def norm(v):
    return math.sqrt(sum(x*x for x in v))


def means(rows):
    return [st.mean(col) for col in zip(*rows)]


def load(folder):
    # 遇到无效原始行立即报错，避免静默跨越坏数据。
    # Fail on invalid rows rather than silently spanning corrupted data.
    rows = []
    for line_no, line in enumerate((folder/'packets.jsonl').read_text(encoding='utf-8').splitlines(), 1):
        try:
            r = json.loads(line)
            p = r['packet']
            a = [p['data'][k] for k in FIELDS[:3]]
            w = [p['data'][k] for k in FIELDS[3:]]
            if (r['valid_imu'] is not True or p['type'] != 'imu' or p['version'] != 1
                    or any(type(x) not in (int, float) or not math.isfinite(x) for x in a+w)
                    or any(type(p[k]) is not int or p[k] < 0 for k in ('timestamp_us', 'sequence_number'))):
                raise ValueError('Invalid IMU values')
            rows.append(dict(us=p['timestamp_us'], seq=p['sequence_number'], node=p['node_id'], a=a, w=w))
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(f'Invalid row {line_no}: {error}') from error
    if len(rows) < 2:
        raise ValueError('Need at least two samples')
    for i, r in enumerate(rows):
        r['t'] = (r['us']-rows[0]['us'])/1e6
        if i and (r['us'] <= rows[i-1]['us'] or r['seq'] <= rows[i-1]['seq'] or r['node'] != rows[0]['node']):
            raise ValueError('Time/sequence reset, duplicate, reorder or mixed nodes')
    return rows


def windows(rows, seconds=0.2, max_gap=0.04):
    # 因果窗口：判定时间为窗口末端，缺号/长间隔后重新积累完整窗口。
    # Causal windows: timestamp decisions at the end; restart after missing IDs or long gaps.
    times = [r['t'] for r in rows]
    start = 0
    result = []
    for i, r in enumerate(rows):
        if i and (r['seq'] != rows[i-1]['seq']+1 or r['t']-rows[i-1]['t'] > max_gap):
            start = i
        j = max(start, bisect.bisect_right(times, r['t']-seconds+1e-9, start, i+1)-1)
        result.append((j, i+1) if r['t']-rows[j]['t'] >= seconds-1e-6 else None)
    return result


def features(block, bias):
    # 使用加速度三维波动，而非仅模长波动；不从加速度中扣除各轴均值。
    # Use 3D acceleration variability, not only norm variability; do not bias-correct acceleration here.
    av = means([r['a'] for r in block])
    gyro = math.sqrt(st.mean(norm([x-b for x, b in zip(r['w'], bias)])**2 for r in block))
    accel_var = math.sqrt(st.mean(norm([x-m for x, m in zip(r['a'], av)])**2 for r in block))
    gravity_error = math.sqrt(st.mean((norm(r['a'])-G)**2 for r in block))
    return [gyro, accel_var, gravity_error]


def percentile(values, q):
    values = sorted(values)
    pos = (len(values)-1)*q
    low = int(pos)
    return values[low]+(values[min(low+1, len(values)-1)]-values[low])*(pos-low)


def analyze(rows, train=(5.0, 10.0), validation=(55.0, 60.0)):
    if not (0 <= train[0] < train[1] <= validation[0] < validation[1] <= rows[-1]['t']+0.03):
        raise ValueError('Training and validation must be ordered, disjoint and covered by the recording')
    bounds = windows(rows)
    selected = set()
    training_windows = []
    for bound in bounds:
        if bound is None:
            continue
        j, end = bound
        if rows[j]['t'] < train[0] or rows[end-1]['t'] >= train[1]:
            continue
        block = rows[j:end]
        # 保守筛选候选静止窗口；这些工程阈值不是最终步态验收标准。
        # Screen candidate still windows conservatively; these are provisional engineering thresholds.
        if all(x <= limit for x, limit in zip(features(block, [0, 0, 0]), [0.15, 0.25, 0.35])):
            selected.update(range(j, end))
            training_windows.append(bound)
    if len(selected) < 100:
        raise ValueError('Fewer than 100 screened calibration samples; inspect the selected interval')
    bias = means([rows[i]['w'] for i in sorted(selected)])
    ftrain = [features(rows[j:end], bias) for j, end in training_windows]
    # 只用训练段设置阈值，验证段不参与拟合。
    # Fit thresholds only on training data; never use validation data for fitting.
    thresholds = [max(floor, 1.5*percentile([f[k] for f in ftrain], 0.99))
                  for k, floor in enumerate([0.06, 0.15, 0.20])]
    result = []
    for r, bound in zip(rows, bounds):
        f = features(rows[bound[0]:bound[1]], bias) if bound else None
        result.append(dict(t=r['t'], sequence=r['seq'], features=f,
                           state='unknown' if f is None else ('candidate' if all(x <= lim for x, lim in zip(f, thresholds)) else 'moving')))
    vrows = [r for r in rows if validation[0] <= r['t'] < validation[1]]
    if len(vrows) < 100:
        raise ValueError('Insufficient independent validation data')
    before = means([r['w'] for r in vrows])
    after = [x-b for x, b in zip(before, bias)]
    vr = [r for r in result if validation[0] <= r['t'] < validation[1]]
    eligible = [r for r in vr if r['state'] != 'unknown']
    report = dict(train_s=train, validation_s=validation, screened_samples=len(selected),
                  bias_radps=bias, thresholds=dict(zip(['gyro_rms_radps', 'accel_vector_rms_mps2', 'gravity_error_rms_mps2'], thresholds)),
                  screening_limits=[0.15, 0.25, 0.35], window_s=0.2, max_gap_s=0.04,
                  validation=dict(n=len(vrows), mean_before_radps=before, mean_after_radps=after,
                                  bias_mean_norm_before=norm(before), bias_mean_norm_after=norm(after),
                                  gyro_rms_before=math.sqrt(st.mean(norm(r['w'])**2 for r in vrows)),
                                  gyro_rms_after=math.sqrt(st.mean(norm([x-b for x,b in zip(r['w'],bias)])**2 for r in vrows)),
                                  eligible_windows=len(eligible), unknown_windows=len(vr)-len(eligible),
                                  candidate_fraction=sum(r['state']=='candidate' for r in eligible)/len(eligible) if eligible else None),
                  total_samples=len(rows), missing_ids=rows[-1]['seq']-rows[0]['seq']+1-len(rows),
                  unknown_windows=sum(r['state']=='unknown' for r in result),
                  note='Candidate detection only; no velocity/position or ZUPT update. Ratios are not accuracy. IMU alone cannot prove zero velocity.')
    return report, result


def show(report, result, check=False):
    import tkinter as tk
    from tkinter import ttk
    root = tk.Tk()
    if check:
        root.withdraw()
    root.title('ZUPT 候选静止检测 / Candidate stationary detection')
    root.geometry('1100x850')
    root.minsize(800, 700)
    v = report['validation']
    ttk.Label(root, padding=12, text='5–10 秒筛选零偏 → 55–60 秒独立验证 → 全段候选静止检测\n'
              'Bias screening → Independent validation → Candidate detection', font=('Microsoft YaHei UI', 12)).pack(fill='x')
    ttk.Label(root, padding=(12, 0), wraplength=760, text=f"零偏 / Bias (rad/s): {', '.join(f'{x:.5f}' for x in report['bias_radps'])}\n"
              f"验证均值向量模长 / Validation mean norm: {v['bias_mean_norm_before']:.5f} → {v['bias_mean_norm_after']:.5f} rad/s\n"
              '绿色 = 候选静止；灰色 = 数据不足/缺号后窗口；虚线 = 阈值。\n'
              'Green = stationary candidate; gray = insufficient data / window after a sequence gap; dashed line = threshold.\n'
              '均为窗口末端判定，非真实落脚标签。\n'
              'Decisions are timestamped at the window end; they are not ground-truth foot-contact labels.').pack(fill='x')
    titles = [('角速度 RMS / Gyro RMS', 'rad/s'), ('加速度三维波动 / Acceleration variation', 'm/s²'), ('重力模长偏差 RMS / Gravity magnitude error', 'm/s²')]
    canvases = [tk.Canvas(root, background='white', highlightthickness=0) for _ in titles]
    for c in canvases:
        c.pack(fill='both', expand=True, padx=12, pady=4)
    detail = ttk.Label(root, padding=8, text='鼠标查看窗口判定 / Hover to inspect window decisions')
    detail.pack(fill='x')
    ttk.Label(root, padding=8, wraplength=760, text='仅离线候选检测，不是 ZUPT 速度更新或轨迹。阈值为本记录初始值，步态检出率尚未验证。\n'
              'Offline candidates only, not ZUPT velocity updates or a trajectory. Thresholds are initial values for this recording; gait detection performance is unverified.').pack(fill='x')
    limits = list(report['thresholds'].values())
    times = [r['t'] for r in result]

    def draw(event=None):
        for k, c in enumerate(canvases):
            c.delete('all')
            width, height = c.winfo_width(), c.winfo_height()
            if width < 150 or height < 100:
                continue
            left, right, top, bottom = 80, width-24, 32, height-35
            ceiling = max(limits[k], max(r['features'][k] for r in result if r['features']))*1.08
            x = lambda t: left+t/times[-1]*(right-left)
            y = lambda val: bottom-val/ceiling*(bottom-top)
            for i, r in enumerate(result):
                if r['state'] in ('candidate', 'unknown'):
                    end = times[i+1] if i+1<len(times) else times[i]
                    c.create_rectangle(x(r['t']), top, x(end), bottom, fill='#d8efdf' if r['state']=='candidate' else '#e7e7e7', outline='')
            c.create_text(8, 13, anchor='w', text=f'{titles[k][0]} ({titles[k][1]}) · 阈值 / threshold {limits[k]:.4f}')
            for j in range(4):
                val=ceiling*j/3
                c.create_line(left,y(val),right,y(val),fill='#dddddd')
                c.create_text(left-6,y(val),anchor='e',text=f'{val:.3f}')
                ti=times[-1]*j/3
                c.create_text(x(ti),bottom+12,text=f'{ti:.1f}')
            c.create_text((left+right)/2,height-7,text='节点时间 / Node time (s)')
            c.create_line(left,y(limits[k]),right,y(limits[k]),fill='#a34b24',dash=(4,3))
            points=[]
            for r in result:
                if r['features'] is None:
                    if len(points)>=4:c.create_line(*points,fill='#176b9b',width=1.2)
                    points=[]
                else:points.extend((x(r['t']),y(r['features'][k])))
            if len(points)>=4:c.create_line(*points,fill='#176b9b',width=1.2)

    def hover(event):
        ti=max(0,min(1,(event.x-80)/max(1,event.widget.winfo_width()-104)))*times[-1]
        r=result[min(bisect.bisect_left(times,ti),len(result)-1)]
        labels={'candidate':'候选静止 / Candidate', 'moving':'未通过静止判据 / Non-stationary criteria', 'unknown':'未知 / Unknown'}
        values='—' if r['features'] is None else ', '.join(f'{v:.4f}' for v in r['features'])
        detail.config(text=f"t={r['t']:.3f} s | {labels[r['state']]} | 特征 / Features: {values}")
    for c in canvases:
        c.bind('<Configure>',draw)
        c.bind('<Motion>',hover)
    if check:
        # 隐藏窗口无法取得可见布局尺寸，使用指定尺寸检查绘图对象。
        # Hidden windows lack visible layout dimensions; use explicit sizes to check drawing objects.
        root.update_idletasks()
        for c in canvases:
            c.winfo_width=lambda:1050
            c.winfo_height=lambda:185
        draw()
        assert all(len(c.find_all())>20 for c in canvases)
        root.destroy()
        return
    root.mainloop()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session',type=Path,default=DEFAULT)
    parser.add_argument('--no-gui',action='store_true')
    args=parser.parse_args()
    try:
        rows=load(args.session)
        report,result=analyze(rows)
    except (OSError,ValueError) as error:
        parser.error(str(error))
    report['source']=str(args.session.resolve())
    report['source_sha256']=hashlib.sha256((args.session/'packets.jsonl').read_bytes()).hexdigest()
    # 每次输出独立目录，保留原始数据及历史分析。
    # Write each run to a new directory, preserving raw data and previous analyses.
    out=ROOT/'analysis'/('zupt_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    out.mkdir(parents=True)
    (out/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    (out/'detections.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in result),encoding='utf-8')
    print(json.dumps(report,ensure_ascii=True,indent=2))
    print('Saved:',out)
    if not args.no_gui:show(report,result)


if __name__=='__main__':
    main()
