"""实时录制的六图诊断，保留真实状态与接收时间。 / Six-panel live-recording diagnostic using recorded states and arrival times."""
import argparse
import hashlib
import json
import math
from datetime import datetime, timezone, timedelta
from pathlib import Path
from zupt_offline import ROOT
from train_zupt import KEYS

DEFAULT=ROOT/'live_recordings/20261011_034935_192932_live'
STATUS={'unknown':'未知 / Unknown','confirming':'确认中 / Confirming',
        'stationary_candidate':'候选静止 / Candidate',
        'not_stationary_candidate':'未满足 / Not satisfied'}


def prepare(folder, start=33, end=36):
    # 使用日志中的电脑接收相对时间，与终端显示一致；不混用节点时间。
    # Use logged host-relative arrival times to match terminal output, not node-relative time.
    if not 0<=start<end:raise ValueError('Invalid display interval')
    model=json.loads((folder/'model.json').read_text(encoding='utf-8'))
    events=[json.loads(l) for l in (folder/'states.jsonl').read_text(encoding='utf-8').splitlines()]
    measurements=[s for s in events if s['event']=='measurement']
    if not measurements or start<measurements[0]['elapsed_s'] or end>measurements[-1]['elapsed_s']:
        raise ValueError('Display interval is not covered by the recording')
    states=[s for s in events if s.get('elapsed_s') is not None and start<=s['elapsed_s']<=end]
    raw={}
    for l in (folder/'packets.jsonl').read_text(encoding='utf-8').splitlines():
        r=json.loads(l)
        if r.get('accepted_for_detection'):
            key=r['arrival_monotonic_ns']
            if key in raw:raise ValueError('Duplicate arrival timestamp')
            raw[key]=r['packet']
    traces=[[] for _ in range(6)]
    transitions=[];previous=None
    for s in events:
        if start<= (s.get('elapsed_s') if s.get('elapsed_s') is not None else -1)<=end and s['state']!=previous:
            transitions.append(dict(t=s['elapsed_s'],state=s['state'],reason=s['reason']))
        previous=s['state']
    for s in states:
        if s['event']!='measurement':continue
        p=raw[s['arrival_monotonic_ns']]
        if p['sequence_number']!=s['sequence_number'] or p['timestamp_us']!=s['timestamp_us']:
            raise ValueError('State/raw alignment mismatch')
        for i,k in enumerate(('gx_radps','gy_radps','gz_radps')):
            v=p['data'][k]
            traces[i].append((s['elapsed_s'],v,v-model['bias_radps'][i]))
        if s['features'] is not None:
            for i,v in enumerate(s['features']):traces[i+3].append((s['elapsed_s'],v))
    report=dict(session=folder.name,range_s=[start,end],time_basis='host elapsed_s from states.jsonl',
                transitions=transitions,bias_radps=model['bias_radps'],thresholds=model['thresholds'],
                gyro_raw_abs_peak_radps={axis:max(abs(v[1]) for v in traces[i]) for i,axis in enumerate('XYZ')},
                gyro_corrected_abs_peak_radps={axis:max(abs(v[2]) for v in traces[i]) for i,axis in enumerate('XYZ')},
                note='User confirmed stillness near 34.5 s only; plotted states are not ground truth for the entire displayed interval.',
                hashes={name:hashlib.sha256((folder/name).read_bytes()).hexdigest() for name in ('model.json','states.jsonl','packets.jsonl')})
    return model,states,traces,report


def show(model,states,traces,report,check=False):
    import tkinter as tk
    from tkinter import ttk
    root=tk.Tk()
    if check:root.withdraw()
    root.title('实时记录诊断 / Live recording diagnostic');root.geometry('1250x900');root.minsize(950,760)
    lo,hi=report['range_s']
    ttk.Label(root,padding=10,text=f"{report['session']} · {lo:g}–{hi:g} s\n"
        '左：三轴原始/补偿角速度；右：已记录的检测特征。参数不变。\n'
        'Left: raw/corrected gyro axes. Right: recorded detector features. Parameters unchanged.').pack(fill='x')
    ttk.Label(root,padding=(10,0),wraplength=1150,text=
        '橙线：原始；蓝线：补偿后/检测特征；虚线：阈值。底色：绿=候选静止，黄=确认中，红=未满足，灰=未知。\n'
        'Orange: raw; blue: corrected/features; dashed: threshold. Background: green=candidate, yellow=confirming, red=not satisfied, gray=unknown.').pack(fill='x')
    changes=' | '.join(f"{r['t']:.3f}s {STATUS[r['state']]}" for r in report['transitions'])
    ttk.Label(root,padding=8,wraplength=1150,text='状态切换 / Transitions: '+changes).pack(fill='x')
    grid=ttk.Frame(root);grid.pack(fill='both',expand=True)
    canvases=[]
    for i in range(6):
        c=tk.Canvas(grid,bg='white',highlightthickness=0)
        c.grid(row=i%3,column=i//3,sticky='nsew',padx=8,pady=5);canvases.append(c)
    for i in range(3):grid.rowconfigure(i,weight=1)
    for i in range(2):grid.columnconfigure(i,weight=1)
    detail=ttk.Label(root,padding=8,text='鼠标查看数值 / Hover to inspect values');detail.pack(fill='x')
    ttk.Label(root,padding=8,wraplength=1150,text=
        '横轴为电脑接收相对时间，与终端计时一致。三轴图不应用角速度 RMS 阈值；阈值仅适用于右侧窗口特征。\n'
        'X axes use host elapsed time, matching the terminal. RMS thresholds apply only to window features, not individual gyro axes.').pack(fill='x')
    titles=['陀螺仪 X / Gyro X (rad/s)','陀螺仪 Y / Gyro Y (rad/s)','陀螺仪 Z / Gyro Z (rad/s)',
            '角速度 RMS / Gyro RMS (rad/s)','加速度波动 / Acceleration variation (m/s²)','重力模长偏差 / Gravity error (m/s²)']
    colors={'unknown':'#e5e5e5','confirming':'#fff1c6','stationary_candidate':'#dcf0e1','not_stationary_candidate':'#fae0e0'}
    limits=[model['thresholds'][k] for k in KEYS]
    def draw(event=None):
        for k,c in enumerate(canvases):
            c.delete('all');w,h=c.winfo_width(),c.winfo_height()
            if w<150 or h<120:continue
            left,right,top,bottom=70,w-16,42,h-38
            vals=[v for r in traces[k] for v in r[1:]]+[0]+([limits[k-3]] if k>=3 else [])
            low,high=min(vals),max(vals);pad=max((high-low)*.08,.001);low-=pad;high+=pad
            x=lambda t:left+(t-lo)/(hi-lo)*(right-left)
            y=lambda v:bottom-(v-low)/(high-low)*(bottom-top)
            for j,s in enumerate(states):
                stop=min(hi,states[j+1]['elapsed_s']) if j+1<len(states) else hi
                c.create_rectangle(x(s['elapsed_s']),top,x(stop),bottom,fill=colors[s['state']],outline='')
            c.create_text(8,12,anchor='w',text=titles[k])
            if k>=3:c.create_text(8,30,anchor='w',text=f'阈值 / Threshold {limits[k-3]:.5f}')
            for j in range(4):
                v=low+(high-low)*j/3;t=lo+(hi-lo)*j/3
                c.create_line(left,y(v),right,y(v),fill='#ddd')
                c.create_text(left-5,y(v),anchor='e',text=f'{v:.3f}')
                c.create_text(x(t),bottom+12,text=f'{t:.1f}')
            c.create_text((left+right)/2,h-8,text='接收时间 / Receive time (s)')
            for column in (range(1,3) if k<3 else [1]):
                pts=[];prev=None
                for r in traces[k]:
                    if prev is not None and r[0]-prev>.1:
                        if len(pts)>=4:c.create_line(*pts,fill='#c66b20' if k<3 and column==1 else '#176b9b')
                        pts=[]
                    pts.extend((x(r[0]),y(r[column])));prev=r[0]
                if len(pts)>=4:c.create_line(*pts,fill='#c66b20' if k<3 and column==1 else '#176b9b',width=1.3)
            if k>=3:c.create_line(left,y(limits[k-3]),right,y(limits[k-3]),fill='#333',dash=(4,3))
    def hover(e,k):
        t=lo+max(0,min(1,(e.x-70)/max(1,e.widget.winfo_width()-86)))*(hi-lo)
        r=min(traces[k],key=lambda r:abs(r[0]-t))
        s=min(states,key=lambda s:abs(s['elapsed_s']-r[0]))
        detail.config(text=f"t={r[0]:.3f}s | {STATUS[s['state']]} | {titles[k]}: "+' / '.join(f'{v:.5f}' for v in r[1:]))
    for k,c in enumerate(canvases):
        c.bind('<Configure>',draw);c.bind('<Motion>',lambda e,k=k:hover(e,k))
    if check:
        root.update_idletasks()
        for c in canvases:c.winfo_width=lambda:590;c.winfo_height=lambda:210
        draw();assert all(len(c.find_all())>20 for c in canvases)
        root.destroy()
    else:root.mainloop()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session',type=Path,default=DEFAULT)
    parser.add_argument('--start',type=float,default=33)
    parser.add_argument('--end',type=float,default=36)
    parser.add_argument('--no-gui',action='store_true')
    args=parser.parse_args()
    model,states,traces,report=prepare(args.session,args.start,args.end)
    out=ROOT/'analysis'/('live_diagnostic_'+datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S_%f'))
    out.mkdir(parents=True)
    (out/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2));print('Saved:',out)
    if not args.no_gui:show(model,states,traces,report)


if __name__=='__main__':main()
