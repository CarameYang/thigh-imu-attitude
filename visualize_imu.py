"""IMU 六轴交互查看器（仅 Python 标准库）。 / Interactive IMU viewer (standard library only).

运行 / Run: python visualize_imu.py
检查 / Check: python visualize_imu.py --check
"""
import argparse
import json
import math
import statistics
import tkinter as tk
from pathlib import Path
from tkinter import ttk, messagebox

ROOT = Path(__file__).resolve().parent
FIELDS = ('ax_mps2', 'ay_mps2', 'az_mps2', 'gx_radps', 'gy_radps', 'gz_radps')
NAMES = ('加速度 X / Accel X', '加速度 Y / Accel Y', '加速度 Z / Accel Z',
         '角速度 X / Gyro X', '角速度 Y / Gyro Y', '角速度 Z / Gyro Z')


def load_session(folder):
    # 保留节点时间；跳过无效包并明确计数，不修改文件。
    # Preserve node times; count and skip invalid packets without modifying files.
    rows, invalid = [], 0
    with (folder / 'packets.jsonl').open(encoding='utf-8') as stream:
        for number, line in enumerate(stream, 1):
            try:
                record = json.loads(line)
                p = record['packet']
                if not record['valid_imu'] or not isinstance(p, dict):
                    raise ValueError('Invalid packet')
                values = [p['data'][key] for key in FIELDS]
                if not all(type(v) in (int, float) and math.isfinite(v) for v in values):
                    raise ValueError('Nonfinite measurement')
                if any(type(p[k]) is not int or p[k] < 0 for k in ('sequence_number', 'timestamp_us')):
                    raise ValueError('Invalid sequence or timestamp')
                rows.append((p['timestamp_us'], p['sequence_number'], values))
            except (ValueError, KeyError, TypeError):
                invalid += 1
    if len(rows) < 2:
        raise ValueError(f'{folder.name}: 至少需要两个有效样本 / Need at least two valid samples')
    if any(b[0] <= a[0] or b[1] <= a[1] for a, b in zip(rows, rows[1:])):
        raise ValueError('存在重复、乱序或计时复位，请先检查数据 / Duplicate, reordered or reset sequence/time')
    times = [(r[0] - rows[0][0]) / 1e6 for r in rows]
    axes = [[r[2][i] for r in rows] for i in range(6)]
    norms = [math.sqrt(sum(v * v for v in r[2][:3])) for r in rows]
    return dict(name=folder.name, times=times, axes=axes, n=len(rows), invalid=invalid,
                missing=rows[-1][1]-rows[0][1]+1-len(rows),
                means=[statistics.mean(v) for v in axes],
                sds=[statistics.stdev(v) for v in axes],
                norm=statistics.mean(norms),
                max_gap=max(b-a for a, b in zip(times, times[1:])))


class Viewer:
    def __init__(self, root, folders):
        self.root, self.folders = root, folders
        root.title('IMU 数据查看器 / IMU data viewer')
        root.geometry('1140x850')
        root.minsize(800, 700)
        self.session = tk.StringVar(value=folders[0].name)
        self.preview = tk.BooleanVar(value=False)
        self.average = tk.BooleanVar(value=True)
        controls = ttk.Frame(root, padding=12)
        controls.pack(fill='x')
        ttk.Label(controls, text='录制 / Recording').grid(row=0, column=0, sticky='w')
        choice = ttk.Combobox(controls, textvariable=self.session, state='readonly',
                              values=[f.name for f in folders], width=48)
        choice.grid(row=0, column=1, sticky='w', padx=8)
        choice.bind('<<ComboboxSelected>>', self.select)
        ttk.Checkbutton(controls, text='陀螺仪减均值预览 / Gyro mean subtraction',
                        variable=self.preview, command=self.draw).grid(row=1, column=0, columnspan=2, sticky='w')
        ttk.Checkbutton(controls, text='每 10 点均值 / 10-sample means', variable=self.average,
                        command=self.draw).grid(row=2, column=0, columnspan=2, sticky='w')
        self.summary = ttk.Label(root, padding=(12, 0), wraplength=1000)
        self.summary.pack(fill='x')
        grid = ttk.Frame(root, padding=8)
        grid.pack(fill='both', expand=True)
        self.canvases = []
        # 每行比较同一轴的加速度与角速度。 / Pair acceleration and angular rate of each axis per row.
        for i in range(6):
            canvas = tk.Canvas(grid, background='#ffffff', highlightthickness=0)
            canvas.grid(row=i % 3, column=i // 3, sticky='nsew', padx=5, pady=5)
            canvas.bind('<Configure>', lambda event: self.draw())
            canvas.bind('<Motion>', lambda event, axis=i: self.hover(event, axis))
            self.canvases.append(canvas)
        for row in range(3):
            grid.rowconfigure(row, weight=1)
        for col in range(2):
            grid.columnconfigure(col, weight=1)
        self.detail = ttk.Label(root, padding=(12, 4), text='移动鼠标查看时间附近的原始样本 / Hover for nearest raw sample')
        self.detail.pack(fill='x')
        ttk.Label(root, padding=(12, 8), wraplength=1050,
                  text='统计使用全部有效原始样本；SD 为样本标准差。显示平均不改变原始数据。\n'
                       '减去本记录自身均值仅为演示，不是独立校准验证；运动均值不能当作静止零偏。\n'
                       'Full-sample statistics; display averaging only. Self-mean subtraction is not independent calibration.').pack(fill='x')
        self.data = None
        self.select()

    def select(self, event=None):
        try:
            self.data = load_session(next(f for f in self.folders if f.name == self.session.get()))
            self.preview.set(False)
            self.draw()
        except (OSError, ValueError) as error:
            self.data = None
            for canvas in self.canvases:
                canvas.delete('all')
            self.summary.config(text=str(error))
            messagebox.showerror('读取失败 / Read failed', str(error))

    def draw(self):
        if not getattr(self, 'data', None):
            return
        d = self.data
        self.summary.config(text=f"{d['name']}  |  {d['n']} 样本 / samples  |  缺号 / missing {d['missing']}  |  无效 / invalid {d['invalid']}\n"
                                 f"加速度合成均值 / Mean |a|: {d['norm']:.5f} m/s²  |  最大节点间隔 / Max node gap: {d['max_gap']*1000:.3f} ms")
        for i, canvas in enumerate(self.canvases):
            canvas.delete('all')
            w, h = canvas.winfo_width(), canvas.winfo_height()
            if w < 150 or h < 100:
                continue
            left, top, right, bottom = 78, 52, w-20, h-38
            bias = d['means'][i] if i >= 3 and self.preview.get() else 0
            raw = d['axes'][i]
            # 固定原始/补偿共同纵轴范围，避免自动缩放掩盖变化。
            # Share raw and corrected y limits so autoscaling does not hide the shift.
            candidates = raw + ([v-d['means'][i] for v in raw] if i >= 3 else []) + [0]
            lo, hi = min(candidates), max(candidates)
            pad = max((hi-lo)*0.08, 0.001)
            lo, hi = lo-pad, hi+pad
            xmap = lambda t: left+t/d['times'][-1]*(right-left)
            ymap = lambda v: bottom-(v-lo)/(hi-lo)*(bottom-top)
            unit = 'm/s²' if i < 3 else 'rad/s'
            canvas.create_text(8, 12, anchor='w', text=f'{NAMES[i]} ({unit})', font=('Microsoft YaHei UI', 10, 'bold'))
            canvas.create_text(8, 33, anchor='w', text=f"原始均值 / Mean {d['means'][i]:.5f}    SD {d['sds'][i]:.5f}", font=('Microsoft YaHei UI', 9))
            for j in range(4):
                val = lo+(hi-lo)*j/3
                y = ymap(val)
                canvas.create_line(left, y, right, y, fill='#e0e4e8')
                canvas.create_text(left-6, y, anchor='e', text=f'{val:.3f}')
            canvas.create_line(left, ymap(0), right, ymap(0), fill='#8b949e', dash=(3, 3))
            for j in range(4):
                t = d['times'][-1]*j/3
                canvas.create_text(xmap(t), bottom+13, text=f'{t:.1f}')
            canvas.create_text((left+right)/2, h-8, text='节点时间 / Node time (s)')
            step = 10 if self.average.get() else 1
            coords = []
            # 时间与数值按相同分组取均值，间断处不连线。
            # Average times and values over matching blocks; break lines at long gaps.
            for start in range(0, len(raw), step):
                if start and d['times'][start]-d['times'][start-1] > 0.1:
                    if len(coords) >= 4:
                        canvas.create_line(*coords, fill='#176b9b', width=1.2)
                    coords = []
                t = statistics.mean(d['times'][start:start+step])
                v = statistics.mean(raw[start:start+step])-bias
                coords.extend((xmap(t), ymap(v)))
            if len(coords) >= 4:
                canvas.create_line(*coords, fill='#176b9b', width=1.2)

    def hover(self, event, axis):
        if not self.data:
            return
        # 鼠标读数总是显示最近原始样本。 / Hover always reports the nearest raw sample.
        import bisect
        d = self.data
        width = self.canvases[axis].winfo_width()
        t = max(0, min(1, (event.x-78)/max(1, width-98)))*d['times'][-1]
        index = min(bisect.bisect_left(d['times'], t), d['n']-1)
        if index and abs(d['times'][index-1]-t) < abs(d['times'][index]-t):
            index -= 1
        value = d['axes'][axis][index]
        offset = d['means'][axis] if axis >= 3 and self.preview.get() else 0
        unit = 'm/s²' if axis < 3 else 'rad/s'
        self.detail.config(text=f'{NAMES[axis]} | t={d["times"][index]:.3f} s | 原始 / Raw {value:.6f} {unit} | 预览 / Preview {value-offset:.6f} {unit}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--recordings', type=Path, default=ROOT/'recordings')
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    folders = sorted(p.parent for p in args.recordings.glob('*/packets.jsonl'))
    if not folders:
        parser.error(f'找不到录制 / No recordings: {args.recordings}')
    root = tk.Tk()
    if args.check:
        root.withdraw()
    app = Viewer(root, folders)
    if args.check:
        # 隐藏窗口检查数据加载和控件绘制，不启动长期界面。
        # Check loading and drawing in a hidden window without a persistent UI.
        for folder in folders:
            app.session.set(folder.name)
            app.select()
            if app.data is None:
                raise RuntimeError('Session loading failed')
            app.preview.set(True)
            app.average.set(False)
            for canvas in app.canvases:
                canvas.configure(width=500, height=210)
            root.update_idletasks()
            app.draw()
            print(folder.name, app.data['n'], 'samples OK')
        root.destroy()
    else:
        root.mainloop()


if __name__ == '__main__':
    main()
