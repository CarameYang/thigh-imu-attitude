"""静止数据统计，保留原始文件。 / Analyze stationary data without changing originals."""
import json
import math
import statistics as st
from pathlib import Path

ROOT = Path(__file__).resolve().parent
FIELDS = ['ax_mps2', 'ay_mps2', 'az_mps2', 'gx_radps', 'gy_radps', 'gz_radps']

def analyze(folder, label):
    # 仅使用有限的有效六轴数据，异常数据直接报错。 / Require valid finite six-axis data; fail on invalid rows.
    rows = [json.loads(line) for line in (folder / 'packets.jsonl').read_text(encoding='utf-8').splitlines()]
    assert all(r['valid_imu'] for r in rows), 'Invalid packets require separate inspection'
    packets = [r['packet'] for r in rows]
    assert all(math.isfinite(p['data'][k]) for p in packets for k in FIELDS)
    t0 = packets[0]['timestamp_us']
    seq = [p['sequence_number'] for p in packets]
    values = [[p['data'][k] for p in packets] for k in FIELDS]
    norm = [math.sqrt(sum(p['data'][k] ** 2 for k in FIELDS[:3])) for p in packets]
    # 所有统计使用完整记录；显示曲线按十点计算均值。 / Statistics use all samples; plotted traces use ten-sample means.
    stats = [{'field': k, 'mean': st.mean(v), 'sd': st.stdev(v)} for k, v in zip(FIELDS, values)]
    trace = []
    for start in range(0, len(packets), 10):
        block = packets[start:start + 10]
        trace.append([st.mean((p['timestamp_us'] - t0) / 1e6 for p in block)] +
                     [st.mean(p['data'][k] for p in block) for k in FIELDS])
    return dict(label=label, source=folder.name, n=len(packets),
                missing=max(seq)-min(seq)+1-len(set(seq)), stats=stats,
                norm_mean=st.mean(norm), norm_sd=st.stdev(norm), trace=trace)

if __name__ == '__main__':
    result = [analyze(ROOT / 'recordings' / name, label) for name, label in [
        ('20260923_024502_927703_stationary', '平放'),
        ('20260923_025602_464673_stationary', '立放')]]
    output = ROOT / 'analysis'
    output.mkdir(exist_ok=True)
    (output / 'stationary_stats.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    for session in result:
        print(session['label'], 'samples:', session['n'], 'missing:', session['missing'])
        for axis in session['stats']:
            print(axis['field'], 'mean:', round(axis['mean'], 6), 'SD:', round(axis['sd'], 6))
        print('Acceleration magnitude:', round(session['norm_mean'], 6), 'm/s^2')
