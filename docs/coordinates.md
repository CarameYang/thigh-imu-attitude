# 坐标系、安装方向与四元数约定
# Coordinate Frames, Mounting Convention and Quaternions

**这是本仓库最容易出错、也最容易产生"看起来正常但符号错了"的一节。**
改动任何轴向、符号或四元数顺序前，请先读完本文并运行 `test_attitude.py`。

This is the section most likely to hide a silent sign error. Read it fully
before changing any axis, sign or quaternion order.

---

## 1. 三套坐标系 / Three frames

| 坐标系 Frame | 定义 Definition | 用途 Usage |
|---|---|---|
| **传感器系** sensor（丝印系） | 模块丝印标注的原始轴；固件原样传输，**不做任何交换或取反** | 原始数据、`R_BS` 的输入侧 |
| **身体系** body | `+x` 前、`+y` 左、`+z` 上（右手系） | 安装映射的出口；姿态与人体语义对应的坐标系 |
| **世界系** world | `+z` 向上；`+x` / `+y` 的水平朝向在初始化时选定 | 去重力、速度、位置的输出坐标系 |

世界系的水平朝向**不是真北**：六轴没有绝对航向参考，初始化时选定的
`+x` 方向只是"当时的某个水平方向"。因此世界系航向会随陀螺零偏漂移。
The world frame is **not** north-aligned: a six-axis IMU has no absolute
heading reference, so the chosen horizontal direction drifts with gyro bias.

---

## 2. 安装约定（用户确认）/ Mounting convention (user-confirmed)

佩戴方式：**丝印面朝腿外**时 / With the silkscreen face pointing away from the leg:

| 传感器轴 Sensor axis | 物理方向 Physical direction |
|---|---|
| `+X` | 向上 up |
| `+Y` | 向左 left |
| `+Z` | 向后 back（朝大腿后方 / toward the back of the thigh） |

安装方式为**大腿佩戴**（thigh-mounted）；手持录制的朝向由手决定，
**不能作为安装依据**。
Recordings made hand-held do not define the worn orientation.

### 安装矩阵 / Mounting matrix `R_BS`

`attitude.R_BS` 把**传感器系向量**旋转到**身体系**（行 = 身体轴，列 = 传感器轴）：

```python
# attitude.py
R_BS = ((0.0, 0.0, -1.0),     # body x  = −sensor Z
        (0.0, 1.0, 0.0),      # body y  =  sensor Y
        (1.0, 0.0, 0.0))      # body z  =  sensor X
```

等价写法 / Equivalent form：

```text
body = (−sensor Z, sensor Y, sensor X)          #  即 body_from_sensor()

sensor +X → body +z        （上   → 上）
sensor +Y → body +y        （左   → 左）
sensor +Z → body −x        （后   → 后，因为 body +x 是前）
```

**右手性校验 / Handedness check**：`det(R_BS) = +1`（`test_attitude.py` 中有断言）。
若行列式为 −1，说明映射里混入了镜像，必须停下排查。
A determinant of −1 means a mirror was introduced somewhere.

### 这个映射是怎么定下来的 / How this was established

1. 用户确认丝印面朝腿外时的物理朝向（上表）。
2. 用真实佩戴站立段的**比力实测**交叉验证（站立时比力方向应指向 +z）。
3. 站立窗口实测安装倾角约 **6.71° / 6.87° / 7.90°**，与"基本竖直但不精密"一致。
   依据见 `analysis/attitude_validation_20261012/` 与
   `analysis/mounting_check_20261012_003433/`。

> **名义映射 ≠ 精密安装角标定。** `R_BS` 只回答"哪根轴对应哪根轴"；
> 真实的佩戴倾角由姿态估计（`q_bw`）表示，不写进 `R_BS`。
> `R_BS` is a nominal axis mapping; actual mounting tilt is carried by `q_bw`.

> **注意不可观测的自由度**：绕**传感器 x 轴**的 180° 旋转在六轴下不可观测
> （重力方向相同、角速度符号在水平轴上会变但静止时无法区分）。
> 需要磁力计或 UWB 复核。
> A 180° rotation about the sensor x axis is **not observable** from six axes.

---

## 3. 四元数约定 / Quaternion convention

| 项目 Item | 约定 Convention |
|---|---|
| 存储顺序 Storage order | `(w, x, y, z)` —— **w 在前** |
| 乘法 Product | Hamilton 约定（`quat_multiply`） |
| 语义 Semantics | `q_bw` 表示**身体到世界**的旋转 / body → world |
| 归一化 | `quat_normalize()`；`q` 与 `−q` 表示同一旋转 |
| 欧拉角 | `euler_from_quat()` 仅用于展示，不用于积分 |

**易错点 / Common trap**：`q_bw` 与其逆 `q_wb` 差一个共轭。向量旋转方向写反
时，静止段的去重力结果会出现约 2 g 的错误量级，而不是微小偏差——如果看到
"静止时线加速度接近 ±19.6 m/s²"，先查这里。
Reversing the rotation direction produces a ~2 g error at rest, not a small bias.

---

## 4. 单位 / Units

全仓库内部统一使用 **SI 单位**，不混用度与弧度：
All modules use SI internally; degrees never enter the pipeline.

| 量 Quantity | 单位 Unit |
|---|---|
| 位置 / 距离 | m |
| 速度 | m/s |
| 加速度、比力 | m/s² |
| 角速度 | rad/s |
| 角度（仅展示层） | rad（`euler_from_quat` 输出） |
| 时间戳 | 微秒整数 µs（`timestamp_us`） |
| 时间间隔（算法内部） | s |

固件侧的换算（只读核对，不在本仓库代码内）：
Firmware-side conversion (read-only verified, not in this repository):

```text
加速度: 库输出 mg × 0.00980665      → m/s²
角速度: 库输出 deg/s × 0.01745329252 → rad/s
```

---

## 5. 时间语义 / Time semantics

| 时间 Time | 来源 Source | 用途 Usage |
|---|---|---|
| `timestamp_us` | **ESP32 节点**时钟，整数微秒 | **姿态与速度积分只用它** |
| `arrival_monotonic_ns` | 电脑单调时钟 | 记录、超时判断、诊断 |
| `arrival_unix_ns` | 电脑墙上时钟 | 仅记录 |
| 窗口秒数（如 `8.5–10 s`） | **电脑接收相对时间**，0 = 首包 | 报告与图表标注 |

**这三种时间没有被当作已经同步。** 节点时间近似为"读取开始时刻"，不是传感器
内部硬件采样时间戳；跨节点同步、端到端延迟、内部 ODR 均未验证。
These clocks are **not** synchronized, and cross-node sync, end-to-end latency
and the sensor's internal ODR remain unverified.

> 实测参考：节点相邻间隔约 **19.556–20.449 ms**（约 50 Hz），而电脑相邻到达
> 间隔曾出现约 **221 ms** 的抖动。因此积分必须用节点时间差。
> Node intervals were ~19.556–20.449 ms while host arrival showed a ~221 ms
> gap, which is why integration uses node time.

---

## 6. 去重力公式 / Gravity removal

```text
a_world = R_bw · R_BS · f_sensor + (0, 0, −9.80665)
          └────┬────┘  └──┬──┘  └───┬───┘   └──────┬──────┘
          姿态(身体→世界) 安装  传感器比力   重力向量（向下）
```

逐项含义 / Step by step：

1. `R_BS · f_sensor`：把传感器比力转到身体系。
2. `R_bw · (…)`：按当前姿态转到世界系。
3. `+ (0, 0, −9.80665)`：加入向下的重力向量。

**为什么静止时不是零？** 静止的加速度计测到的是**比力**：桌面支撑使它读到约
`(0, 0, +9.81)`，与重力向量相加后才接近零。自由落体时才接近零比力。
**Why not zero at rest?** An accelerometer measures specific force: at rest it
reads about `(0, 0, +9.81)` in world coordinates, which cancels the added
gravity term. Free fall is the case that approaches zero specific force.

**不能做的事 / Do not:**

- ❌ 固定从某一个原始轴减去 `9.80665`——设备会倾斜。
- ❌ 把原始加速度直接积分。
- ❌ 用重力常数去吸收方向误差（见下）。

**方向误差 vs 幅值误差**：把世界比力写成 `幅值 m × 单位方向 u`，残差可分解为
`(m − g)·u + g·(u − e_z)`。手持窗口 X 均值的分解结果是：幅值项贡献约
**−0.00174**，方向项约 **−0.17969** m/s²。也就是说该持续偏差**主要来自方向
不一致，调重力常数无法消除**。
The persistent X offset in the hand-held window comes overwhelmingly from the
direction term, so tuning the gravity constant cannot fix it.

---

## 7. 展示层注意 / Presentation-layer notes

- 报告中的加速度统计顺序恒为 **世界 X、Y、Z**，单位 m/s²。
- 若在 SVG 或其他"y 轴向下"的画布上绘图，需要**显式**的世界→显示坐标转换
  （`y_display = −y_world`），不要直接把世界坐标当屏幕坐标。
- `visualize_imu.py` 中的平滑曲线只是观察辅助；陀螺"减均值预览"**不是**正式标定。

---

## 8. 常见错误清单 / Common mistakes

| 错误 Mistake | 后果 Symptom |
|---|---|
| 用世界向量直接乘 `q_wb`（方向写反） | 静止时线加速度量级约 ±2 g |
| 忘记 `R_BS`，直接把传感器轴当身体轴 | 前后/上下轴互换，重力残差大且随姿态变化 |
| 用电脑接收时间差做积分 | 网络抖动被当成运动，速度出现尖峰 |
| 四元数写成 `(x, y, z, w)` | 姿态完全错乱 |
| 认为 `det(R_BS) = −1` 也能用 | 隐含镜像，左右或前后翻转 |
| 假设世界系 `+x` 是真北 | 航向结论全部失效 |
| 把 `zupt_allowed` 当成速度已归零 | 速度不收敛，轨迹无意义 |

### 已作废的历史说法 / Superseded descriptions

- 早期资料中"**sensor +X → body −z**"的写法**与当前定义和实际代码不符，不再采用**。
  历史文件保留原始记录，但不得用于后续实施。
- 早期关于佩戴轴向的**口头描述**（在丝印朝向确认之前）一律作废。

---

## 9. 改动检查表 / Change checklist

修改任何坐标、符号或安装相关代码后：

- [ ] `python -m unittest test_attitude` —— 22 项，含 `det(R_BS)` 与积分符号断言。
- [ ] `python -m unittest test_gravity_removal test_velocity_propagation`
- [ ] `python -X utf8 check_pregravity.py` —— 数据与姿态内部一致性。
- [ ] `python -X utf8 validate_gravity_removal.py` —— 静止残差不应系统性变差。
- [ ] 若结论有变，在项目进度记录中写明依据，**不要静默重写历史结论**。
