"""漏检诊断：使用已保存模型，不重新训练。 / Miss diagnostic using a saved model without refitting.
运行 / Run: python diagnose_zupt.py
仅报告 / Report only: python diagnose_zupt.py --no-gui
"""
import argparse
import hashlib
import json
from datetime import datetime, timezone, timedelta
from pathlib import Path
from zupt_offline import ROOT, load, features, norm
from train_zupt import segment, KEYS, evaluate

MODEL = ROOT/'analysis/zupt_training_20261007_164450_519158/model.json'
SESSION = ROOT/'recordings/20261007_155209_788135_motion'


def diagnose(model, rows):
    # 与正式验证使用相同区间及完整窗口，避免统计口径变化。
    # Use the same interval and complete windows as validation for consistent accounting.
    spec=dict(session=SESSION.name,start_s=5,end_s=10)
    indices,bounds=segment(rows,spec,model['detector'])
    limits=[model['thresholds'][k] for k in KEYS]
    samples=[]
    for j,end in bounds:
        f=features(rows[j:end],model['bias_radps'])
        raw=features(rows[j:end],[0,0,0])
        flags=[v>limit for v,limit in zip(f,limits)]
        samples.append(dict(t=rows[end-1]['t'],features=f,raw_gyro_rms=raw[0],failed=flags))
    runs=[]
    for s in samples:
        if any(s['failed']):
            if not runs or runs[-1]['closed'] or s['t']-runs[-1]['last_endpoint_s']>model['detector']['max_gap_s']:
                runs.append(dict(first_endpoint_s=s['t'],last_endpoint_s=s['t'],windows=0,closed=False))
            runs[-1]['last_endpoint_s']=s['t'];runs[-1]['windows']+=1
        elif runs:runs[-1]['closed']=True
    for r in runs:r.pop('closed')
    report=evaluate(model,rows,spec)
    report['miss_runs_window_endpoints']=runs
    report['exclusive_failure_counts']={
        'gyro_only':sum(s['failed']==[True,False,False] for s in samples),
        'accel_only':sum(s['failed']==[False,True,False] for s in samples),
        'gyro_and_accel':sum(s['failed']==[True,True,False] for s in samples)}
    report['gyro_failed_without_bias_same_threshold']=sum(s['raw_gyro_rms']>limits[0] for s in samples)
    report['raw_vs_corrected_gyro_changes']={
        'new_failures':sum(s['raw_gyro_rms']<=limits[0] and s['failed'][0] for s in samples),
        'resolved_failures':sum(s['raw_gyro_rms']>limits[0] and not s['failed'][0] for s in samples)}
    report['note']='User-confirmed stationary labels retained. Counterfactual no-bias results are diagnostic only, not a new fitted model.'
    series=[[(rows[i]['t'],norm(rows[i]['w'])) for i in indices],
            [(rows[i]['t'],norm([a-b for a,b in zip(rows[i]['w'],model['bias_radps'])])) for i in indices]]
    return report,samples,series


def display(model, report, samples, series, check=False):
    import tkinter as tk
    from tkinter import ttk
    root=tk.Tk()
    if check:root.withdraw()
    root.title('静止漏检诊断 / Stationary miss diagnostic')
    root.geometry('1150x900');root.minsize(820,740)
    ttk.Label(root,padding=10,text='20261007_155209 · 5–10 s · 用户确认静止 / User-confirmed stationary\n'
              f"检出 / Detected {report['candidate_windows']}/{report['eligible_windows']} · "
              '固定已训练参数 / Fixed trained parameters').pack(fill='x')
    ttk.Label(root,padding=(10,0),wraplength=1000,text=
        '浅红：漏检窗口末端区间；灰色：区间开始时窗口不足。橙线：原始；蓝线：补偿后；虚线：阈值。\n'
        'Red shading: missed window-end decisions; gray: window warm-up. Orange: raw; blue: corrected; dashed: threshold.').pack(fill='x')
    canvases=[tk.Canvas(root,bg='white',highlightthickness=0) for _ in range(4)]
    for c in canvases:c.pack(fill='both',expand=True,padx=10,pady=4)
    hint=ttk.Label(root,padding=8,text='鼠标查看具体数值 / Hover to inspect values');hint.pack(fill='x')
    ttk.Label(root,padding=8,wraplength=1050,text=
        '第一图是逐样本角速度大小；第二图才是检测器使用的窗口 RMS。时间均为节点时间。\n'
        'Panel 1 shows per-sample angular-rate magnitude; panel 2 shows detector window RMS. Times use the node clock.').pack(fill='x')
    limits=list(model['thresholds'].values())
    datasets=[series, [[(s['t'],s['raw_gyro_rms']) for s in samples],[(s['t'],s['features'][0]) for s in samples]],
              [[(s['t'],s['features'][1]) for s in samples]], [[(s['t'],s['features'][2]) for s in samples]]]
    titles=['角速度大小 / Angular-rate magnitude (rad/s)', '角速度窗口 RMS / Window gyro RMS (rad/s)',
            '加速度三维波动 / Acceleration variation (m/s²)','重力模长偏差 / Gravity magnitude error (m/s²)']

    def draw(event=None):
        for k,c in enumerate(canvases):
            c.delete('all');w,h=c.winfo_width(),c.winfo_height()
            if w<150 or h<100:continue
            left,right,top,bottom=72,w-20,30,h-32
            ceiling=max([v for line in datasets[k] for t,v in line]+([limits[k-1]] if k else [0]))*1.1
            x=lambda t:left+(t-5)/5*(right-left)
            y=lambda v:bottom-v/max(ceiling,1e-6)*(bottom-top)
            c.create_rectangle(x(5),top,x(samples[0]['t']),bottom,fill='#e5e5e5',outline='')
            for i,s in enumerate(samples):
                if any(s['failed']):
                    end=samples[i+1]['t'] if i+1<len(samples) else s['t']
                    c.create_rectangle(x(s['t']),top,x(end),bottom,fill='#f9dcdc',outline='')
            c.create_text(8,12,anchor='w',text=titles[k])
            for j in range(4):
                v=ceiling*j/3
                c.create_line(left,y(v),right,y(v),fill='#dddddd')
                c.create_text(left-5,y(v),anchor='e',text=f'{v:.3f}')
            for t in range(5,11):c.create_text(x(t),bottom+12,text=str(t))
            for n,line in enumerate(datasets[k]):
                coords=[p for t,v in line for p in (x(t),y(v))]
                c.create_line(*coords,fill=('#c66b20' if len(datasets[k])==2 and n==0 else '#176b9b'),width=1.4)
            if k:
                c.create_line(left,y(limits[k-1]),right,y(limits[k-1]),fill='#333333',dash=(4,3))
                c.create_text(right,12,anchor='e',text=f'阈值 / Limit {limits[k-1]:.5f}')

    def hover(e):
        t=5+max(0,min(1,(e.x-72)/max(1,e.widget.winfo_width()-92)))*5
        if t<samples[0]['t']:
            hint.config(text='窗口不足 / Window warm-up');return
        s=min(samples,key=lambda r:abs(r['t']-t))
        names=['角速度 / Gyro','加速度波动 / Accel variation','重力偏差 / Gravity error']
        fail=', '.join(names[i] for i,f in enumerate(s['failed']) if f) or '无 / None'
        hint.config(text=f"t={s['t']:.3f} s | 超阈值 / Exceeded: {fail}")
    for c in canvases:c.bind('<Configure>',draw);c.bind('<Motion>',hover)
    if check:
        root.update_idletasks()
        for c in canvases:c.winfo_width=lambda:1100;c.winfo_height=lambda:165
        draw()
        assert all(len(c.find_all())>15 for c in canvases)
        root.destroy()
    else:root.mainloop()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--no-gui',action='store_true')
    parser.add_argument('--live',action='store_true',help='本次实时记录33–36秒 / Live recording at 33–36 s')
    args=parser.parse_args()
    if args.live:
        from diagnose_live_zupt import DEFAULT,prepare,show
        model,states,traces,report=prepare(DEFAULT)
        print(json.dumps(report,indent=2))
        if not args.no_gui:show(model,states,traces,report)
        return
    model=json.loads(MODEL.read_text(encoding='utf-8'));rows=load(SESSION)
    report,samples,series=diagnose(model,rows)
    report['model_path']=str(MODEL)
    report['model_sha256']=hashlib.sha256(MODEL.read_bytes()).hexdigest()
    report['source_sha256']=hashlib.sha256((SESSION/'packets.jsonl').read_bytes()).hexdigest()
    now=datetime.now(timezone(timedelta(hours=8)))
    out=ROOT/'analysis'/('zupt_diagnostic_'+now.strftime('%Y%m%d_%H%M%S_%f'))
    out.mkdir(parents=True)
    (out/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    (out/'windows.json').write_text(json.dumps(samples,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2));print('Saved:',out)
    if not args.no_gui:display(model,report,samples,series)


if __name__=='__main__':main()
