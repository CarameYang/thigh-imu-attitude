"""多记录静止检测器训练与验证，非 INS/ZUPT 速度更新。
Multi-recording stationary detector fitting and validation, not INS/ZUPT velocity updates.

训练 / Train: python train_zupt.py
应用保存模型 / Apply saved model:
python train_zupt.py --model analysis/zupt_training_<time>/model.json --session <folder>
"""
import argparse
import hashlib
import json
import math
import statistics as st
from datetime import datetime, timezone, timedelta
from pathlib import Path
from zupt_offline import ROOT, load, windows, features, means, norm, percentile

KEYS = ('gyro_rms_radps', 'accel_vector_rms_mps2', 'gravity_error_rms_mps2')


def segment(rows, spec, cfg):
    # 半开时间区间；完整窗口必须落在标签内，不能跨训练/验证边界。
    # Half-open intervals; entire windows must lie inside the label without crossing split boundaries.
    lo = float(spec['start_s'])
    hi = float(spec['end_s']) if spec['end_s'] is not None else math.inf
    if not math.isfinite(lo) or lo < 0 or hi <= lo or (spec['end_s'] is not None and not math.isfinite(hi)):
        raise ValueError('Invalid interval: '+str(spec))
    if lo > rows[-1]['t'] or (math.isfinite(hi) and hi > rows[-1]['t']+0.03):
        raise ValueError('Recording does not cover requested interval: '+str(spec))
    indices = [i for i,r in enumerate(rows) if lo <= r['t'] < hi]
    bounds = [b for b in windows(rows, cfg['window_s'], cfg['max_gap_s'])
              if b is not None and rows[b[0]]['t'] >= lo and rows[b[1]-1]['t'] < hi]
    if not bounds:
        raise ValueError('No complete continuous windows in interval: '+str(spec))
    return indices, bounds


def check_split(split):
    # 拒绝重叠标签和重复区间，避免数据泄漏及重复加权。
    # Reject overlapping labels and duplicate intervals to avoid leakage and duplicate weighting.
    spans = []
    for role in ('training', 'validation'):
        if not split.get(role):
            raise ValueError('Empty split: '+role)
        for s in split[role]:
            lo, hi = s['start_s'], s['end_s'] if s['end_s'] is not None else math.inf
            for name,a,b,old_role in spans:
                if name == s['session'] and max(lo,a) < min(hi,b):
                    raise ValueError(f'Overlapping {old_role}/{role} intervals: {name}')
            spans.append((s['session'],lo,hi,role))
    cfg = split['detector']
    if not (0 < cfg['max_gap_s'] <= cfg['window_s'] and 0 < cfg['training_quantile'] <= 1
            and cfg['threshold_multiplier'] >= 1 and len(cfg['threshold_floors']) == 3
            and all(math.isfinite(v) and v > 0 for v in cfg['threshold_floors'])):
        raise ValueError('Invalid detector configuration')


def fit(training, cfg):
    # 接口只接收训练记录，不接收验证集。先分别统计每个区间。
    # This interface receives training data only. Summarize each interval independently first.
    pieces = []
    for spec, rows in training:
        indices, bounds = segment(rows,spec,cfg)
        # 只用完整连续窗口覆盖的样本估计偏置，不按信号大小删除用户标签。
        # Fit bias on samples covered by complete windows; do not discard labels based on signal magnitude.
        usable = sorted({i for j,end in bounds for i in range(j,end)})
        if len(usable) < 100:
            raise ValueError('Too few continuous training samples: '+spec['session'])
        values = [rows[i]['w'] for i in usable]
        pieces.append((spec, rows, bounds, dict(
            interval=spec, labeled_samples=len(indices), bias_samples=len(usable), eligible_windows=len(bounds),
            gyro_mean_radps=means(values), gyro_sd_radps=[st.stdev(c) for c in zip(*values)],
            acceleration_norm_mean_mps2=st.mean(norm(rows[i]['a']) for i in usable))))
    # 每个训练区间等权，避免两个一分钟记录淹没五秒片段。
    # Weight training intervals equally so minute-long recordings do not dominate the five-second segment.
    bias = means([p[3]['gyro_mean_radps'] for p in pieces])
    spread = [max(p[3]['gyro_mean_radps'][k] for p in pieces)-min(p[3]['gyro_mean_radps'][k] for p in pieces) for k in range(3)]
    summaries, quantiles = [], []
    for spec,rows,bounds,summary in pieces:
        values = [features(rows[j:end],bias) for j,end in bounds]
        q = [percentile([f[k] for f in values],cfg['training_quantile']) for k in range(3)]
        summary['feature_quantiles'] = dict(zip(KEYS,q))
        summary['residual_mean_radps'] = [a-b for a,b in zip(summary['gyro_mean_radps'],bias)]
        summaries.append(summary)
        quantiles.append(q)
    # 各区间分位数取最大值并乘安全系数；不使用验证结果调整阈值。
    # Use the largest per-interval quantile with a margin; never tune thresholds on validation results.
    thresholds = [max(cfg['threshold_floors'][k],cfg['threshold_multiplier']*max(q[k] for q in quantiles)) for k in range(3)]
    return dict(schema_version=1,kind='stationary_detector_not_INS',bias_radps=bias,
                thresholds=dict(zip(KEYS,thresholds)),detector=cfg,training=summaries,
                training_bias_range_radps=spread,
                bias_policy='Equal weight per labeled training interval; provisional shared bias, not proof of cross-session stability',
                threshold_policy='Max of per-interval training quantiles times multiplier, bounded below by configured floors',
                limitations=['Positive stationary labels only: false-positive rate is not evaluated.',
                             'Labels are user-confirmed, not independent motion capture.',
                             'Latest recording appears in both splits with disjoint intervals; not session-independent validation.',
                             'No acceleration bias correction, INS, velocity or position update.'])


def evaluate(model, rows, spec):
    indices, bounds = segment(rows,spec,model['detector'])
    bias = model['bias_radps']
    limits = [model['thresholds'][k] for k in KEYS]
    fs = [features(rows[j:end],bias) for j,end in bounds]
    raw = [rows[i]['w'] for i in indices]
    corrected = [[a-b for a,b in zip(v,bias)] for v in raw]
    hits = sum(all(v<=limit for v,limit in zip(f,limits)) for f in fs)
    return dict(interval=spec,labeled_samples=len(indices),eligible_windows=len(bounds),
                excluded_window_endpoints=len(indices)-len(bounds),
                candidate_windows=hits,stationary_detection_fraction=hits/len(bounds),
                mean_before_radps=means(raw),mean_after_radps=means(corrected),
                mean_norm_before_radps=norm(means(raw)),mean_norm_after_radps=norm(means(corrected)),
                rms_before_radps=math.sqrt(st.mean(norm(v)**2 for v in raw)),
                rms_after_radps=math.sqrt(st.mean(norm(v)**2 for v in corrected)),
                failed_windows_by_feature={k:sum(f[i]>limits[i] for f in fs) for i,k in enumerate(KEYS)})


def detect(model, rows):
    # 每个记录重新初始化窗口，不拼接不同节点时间轴。
    # Reset windows for each recording rather than concatenating node clocks.
    cfg=model['detector'];limits=[model['thresholds'][k] for k in KEYS]
    result=[]
    for r,b in zip(rows,windows(rows,cfg['window_s'],cfg['max_gap_s'])):
        f=features(rows[b[0]:b[1]],model['bias_radps']) if b else None
        state='unknown' if f is None else ('candidate' if all(x<=lim for x,lim in zip(f,limits)) else 'not_stationary_candidate')
        result.append(dict(t=r['t'],timestamp_us=r['us'],sequence=r['seq'],features=f,state=state))
    return result


def save_json(path, obj):
    path.write_text(json.dumps(obj,ensure_ascii=False,indent=2),encoding='utf-8')


def save_detections(out, name, model, rows):
    (out/(name+'_detections.jsonl')).write_text(''.join(json.dumps(r)+'\n' for r in detect(model,rows)),encoding='utf-8')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--split',type=Path,default=ROOT/'zupt_split.json')
    parser.add_argument('--recordings',type=Path,default=ROOT/'recordings')
    parser.add_argument('--model',type=Path)
    parser.add_argument('--session',type=Path)
    args=parser.parse_args()
    # 使用上海时间记录每次训练，并保留历史输出。
    # Timestamp each run in Shanghai time and retain previous outputs.
    now=datetime.now(timezone(timedelta(hours=8)))
    out=ROOT/'analysis'/('zupt_training_'+now.strftime('%Y%m%d_%H%M%S_%f'))
    try:
        if args.model:
            if not args.session:raise ValueError('--model requires --session')
            model=json.loads(args.model.read_text(encoding='utf-8'))
            rows=load(args.session)
            out.mkdir(parents=True)
            save_detections(out,args.session.name,model,rows)
            save_json(out/'provenance.json',dict(model=str(args.model.resolve()),
                model_sha256=hashlib.sha256(args.model.read_bytes()).hexdigest(),
                source=str(args.session.resolve()),source_sha256=hashlib.sha256((args.session/'packets.jsonl').read_bytes()).hexdigest()))
        else:
            if args.session:raise ValueError('--session is only for applying --model')
            split=json.loads(args.split.read_text(encoding='utf-8'));check_split(split)
            names=sorted({s['session'] for role in ('training','validation') for s in split[role]})
            data={n:load(args.recordings/n) for n in names}
            model=fit([(s,data[s['session']]) for s in split['training']],split['detector'])
            validation=[evaluate(model,data[s['session']],s) for s in split['validation']]
            out.mkdir(parents=True)
            model['created_at']=now.isoformat()
            model['sources']={n:dict(path=str((args.recordings/n).resolve()),sha256=hashlib.sha256((args.recordings/n/'packets.jsonl').read_bytes()).hexdigest()) for n in names}
            save_json(out/'model.json',model);save_json(out/'split.json',split)
            save_json(out/'validation.json',validation)
            for n in names:save_detections(out,n,model,data[n])
            lines=['训练完成 / Training complete', 'Bias (rad/s): '+str(model['bias_radps']),
                   'Thresholds: '+str(model['thresholds']), '',
                   '各验证区间仅计算完整窗口的静止检出比例，不是总体准确率。',
                   'Fractions cover eligible stationary windows only, not overall accuracy.']
            for v in validation:
                lines.append(f"{v['interval']['session']}: {v['candidate_windows']}/{v['eligible_windows']} "
                             f"({v['stationary_detection_fraction']:.2%}); excluded={v['excluded_window_endpoints']}; "
                             f"mean norm {v['mean_norm_before_radps']:.6f} -> {v['mean_norm_after_radps']:.6f} rad/s")
            save_json(out/'run_info.json',dict(created_at=now.isoformat(),mounting_latest='thigh',label_source=split['label_source']))
            (out/'summary.txt').write_text('\n'.join(lines),encoding='utf-8')
            print('\n'.join(lines))
        print('输出 / Saved:',out)
    except (ValueError,OSError,KeyError) as error:
        parser.error(str(error))


if __name__=='__main__':
    main()
