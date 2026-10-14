# 算法说明 / Algorithms

本文说明每个算法阶段**实际做了什么、依据什么、以及不能声称什么**。
This document states what each algorithm stage actually does, on what
evidence, and what it must not be claimed to prove.

---

## 0. 总览 / Pipeline overview

```text
原始六轴包
   │
   ├─① 陀螺零偏标定 ──────── 用确认静止段的三轴角速度均值估计偏置
   │
   ├─② 姿态估计 ─────────── Mahony 四元数互补滤波（陀螺预测 + 重力慢校正）
   │                        输出 q_bw（身体 → 世界）
   │
   ├─③ 去重力 ───────────── a_world = R_bw · R_BS · f_sensor + (0,0,−9.80665)
   │                        输出世界系线加速度（纯函数，无状态）
   │
   ├─④ 静止候选检测 ─────── 0.2 s 滑窗 3 个特征 → 阈值 → 0.5 s 持续确认
   │                        输出 zupt_allowed（候选许可，不是速度更新）
   │
   └─⑤ 速度传播 ─────────── 世界系单步梯形积分（无 ZUPT、无位置）
```

**每一级的输出都是下一级的输入，且都可以单独检查。** 采集问题、姿态问题与
积分问题不会互相掩盖。
Each stage is independently checkable, so acquisition, attitude and
integration problems do not mask one another.

---

## 1. 陀螺零偏标定 / Gyro bias calibration

**问题**：陀螺仪即使完全不动，三轴输出通常也不是严格零，而是有一个固定偏移。
**Problem**: a gyro at rest still outputs a persistent offset.

**做法 / Method**：取一段**用户确认静止**的区间，计算三轴角速度的算术平均，
作为该次会话的零偏估计，在姿态传播前逐轴扣除。
Take a user-confirmed still interval, average each angular-rate axis, subtract
before propagation.

**规则 / Rules：**

- **不同会话不直接沿用同一组零偏**（温度、时间、佩戴都会影响）。
- 零偏在**沿重力方向**的分量不可观测：加速度计看不到绕竖直轴的转动，
  该分量会表现为**纯航向漂移**，必须由静止数据估计。
  The bias component along gravity is unobservable from accelerometer data and
  becomes pure yaw drift.
- 标定窗口的选择会显著影响结果，**不得挑选最接近预期角度的窗口**
  （实测转身估计随窗口为 90.02° / 87.48° / 84.44° / 79.59°）。
- 检测器使用的"共用偏置"与姿态回放按录制分别估计的零偏**不是同一套标定**。

**相关代码**：`attitude.py`（`AttitudeEstimator` 的积分项与标定接口）、
`diagnose_gyro_bias.py`（敏感性诊断）。

---

## 2. 姿态估计 / Attitude estimation

**实现**：`attitude.py` 的 `AttitudeEstimator` —— 四元数互补滤波，
陀螺预测 + 重力慢校正。
**Implementation**: a quaternion complementary filter with slow gravity
correction.

| 参数 Parameter | 默认值 Default | 作用 Role |
|---|---|---|
| `proportional_gain` (`kp`) | 0.5 | 倾角修正速度 / tilt correction rate |
| `integral_gain` (`ki`) | 0.01 | 估计残余零偏 / residual bias estimation |
| `accel_trust_window` | 0.35 | 加速度模长偏离 1 g 时衰减重力修正 |

**流程 / Flow：**

1. `align_to_gravity(acceleration)`：用第一帧静止加速度把初始姿态对齐到重力，
   **只影响倾角**（绕竖直轴的航向无法由重力确定）。
2. `update(timestamp_s, acceleration, angular_rate)`：按角速度推进姿态，
   再用加速度中的重力方向缓慢修正倾角。
3. **明显偏离重力大小时减弱修正**——但该机制**不能排除所有运动加速度污染**。

**边界 / Limits：**

- 重力修正只能约束**垂直于重力的两个方向**；沿重力的零偏成为航向漂移。
- "不跨长间断直接积分"不等于"断流后姿态自动恢复准确"。调用者必须决定
  重新对齐或提供有效状态。
- 六轴**没有绝对航向参考**：世界水平朝向由初始化选定，不是真北。
  站立窗口实测残余航向变化约 **7.7° / 12.3° / 13.5°**。

**测试**：`test_attitude.py`（22 项），覆盖安装变换行列式、积分符号、四元数
运算、物理回归与阶段就绪检查。

---

## 3. 去重力 / Gravity removal

**实现**：`gravity_removal.remove_gravity()` —— **纯函数**。

```text
a_world = R_bw · R_BS · f_sensor + (0, 0, −9.80665)
```

**特性 / Properties：**

- 不联网、不滤波、**不估计加速度偏置**、不计算速度或位置。
- **不改写原始文件**；返回 `GravityRemovedSample`，同时保留
  原始传感器比力、世界比力、世界线加速度与测量时间。
- 输入要求：测量与姿态**同一节点时刻**（`timestamp_us` 相等，非负整数）、
  三轴与四元数数值有限、四元数接近单位长度（范数容差 `1e-6`）、
  重力大小有限且大于零。
- **错误时拒绝计算，不返回看似安全的零。**

> ⚠️ **时间标签正确 ≠ 姿态物理正确。** 姿态有效性由上游保证，
> 本模块只做接口级校验。
> Correct timestamps do not guarantee a physically correct attitude.

**验证 / Validation**：`validate_gravity_removal.py` 在四份录制的 **16 个固定
窗口**上回放，输出 `mean` / `RMS` / `abs_p95` / `abs_peak` / `below_reference`。
标定窗口一律取**较早**的区间，评估用**之后**的窗口，不用后面的静止数据反向
标定之前的结果。

| 结果 Result | 数值 Value |
|---|---|
| 三轴 RMS < 0.2 m/s² 的窗口 | **15 / 16**（诊断参考线，非验收门槛） |
| 手持 `025851` 8.5–10 s：X 均值 / RMS | −0.1814 / 0.2123 m/s² |
| 大腿 `003433` 31–39.5 s：Z 绝对峰值 | 1.6862 m/s² |
| 桌面 `004616` 45–59 s：X 绝对峰值 | 1.0920 m/s² |

残差分解 / Residual decomposition（把世界比力写成 `m·u`）：

```text
残差 = (m − g)·u  +  g·(u − e_z)
        └─幅值项─┘    └─方向项─┘
```

手持窗口 X 均值：幅值项约 **−0.00174**、方向项约 **−0.17969** m/s²
→ **主要来自测量方向与姿态竖直方向的不一致，调重力常数无法消除**。
This does not by itself prove the attitude is wrong: accelerometer error, real
motion acceleration and attitude error cannot be separated without an external
reference.

---

## 4. 静止候选检测 / Stationary candidate detection

**实现**：`zupt_offline.py`（离线特征与阈值）+ `train_zupt.py`（训练）+
`stationary_stream.py`（实时状态机）。**在线与离线共用同一套特征和阈值代码。**

### 4.1 特征 / Features（0.2 s 滑窗，三个量）

| 特征 Key | 定义 | 单位 |
|---|---|---|
| `gyro_rms_radps` | 扣除偏置后角速度模长的 RMS | rad/s |
| `accel_vector_rms_mps2` | 加速度相对**窗口均值**的三维波动 RMS（**不用**各轴均值扣除） | m/s² |
| `gravity_error_rms_mps2` | `RMS(|a| − g)` | m/s² |

### 4.2 阈值 / Thresholds

```text
阈值 = max(各训练区间该特征的 99% 分位数) × 1.5，并取配置下限
下限 threshold_floors = [0.06, 0.15, 0.2]   （对应上面三个特征，顺序一致）
```

**这是工程初始参数，不是"保证有效"的通用值。** 验证结果不好时
**不会自动放宽阈值**。
These are initial engineering choices; poor validation does not relax them.

### 4.3 状态机 / State machine

```text
        ┌──────────┐  窗口未满     ┌───────────┐
        │ unknown  │◄─────────────│ warming_up│
        └────┬─────┘              └───────────┘
             │ 窗口满 & 特征达标
             ▼
       ┌────────────┐  持续 ≥0.5 s   ┌──────────────────────┐
       │ confirming │───────────────►│ stationary_candidate │  zupt_allowed = true
       └────────────┘                └──────────────────────┘
             ▲
             │ 特征超阈值 → not_stationary_candidate

  任意时刻遇到下列情况立即 reset：
    · 时间戳非递增 / 序号不连续 / 相邻间隔 > max_gap_s(0.04 s)   → gap_or_reset
    · 非法测量（时间戳、序号或数值不合法）                        → invalid_measurement
    · 0.5 s 无新有效包（接收端超时必须调用 no_data()）            → no_fresh_data
```

**关键语义 / Key semantics：**

> `zupt_allowed = true` 的含义是 **"允许考虑一次零速度观测的候选信号"**，
> **不是**"速度已经归零"，也**不是**已经执行了 ZUPT。
> A candidate is permission to consider a zero-velocity observation, not an
> update.

- 大腿低动态、双脚未换位置、身体回到原处，**都不能单独证明 IMU 瞬时速度为零**。
- 缺包时**不连接数据间断**（缓冲只保留覆盖完整时间窗所需的最早样本）。
- 超时后**禁止沿用最后一次的绿色状态**。

---

## 5. 训练与验证 / Training and validation

**划分**：见 [`zupt_split.json`](data-formats.md#9-zupt_splitjson--训练验证划分-labeled-split)
与 [ZUPT_TRAINING_README.md](../ZUPT_TRAINING_README.md)。

| 规则 Rule | 说明 |
|---|---|
| 训练区间 | 两份 stationary 全段 + 最新 motion 的 `[55, 60)` s |
| 验证区间 | 四份 motion 各自的 `[5, 10)` s |
| 窗口完整性 | 完整 0.2 s 窗口必须**全部落在标签内**；缺号或 > 0.04 s 间隔会重置窗口 |
| 偏置合并 | 各训练区间**等权**平均，避免长录制主导 |
| 加速度处理 | **不减各轴均值**（避免把倾角误当偏置扣掉） |
| 会话拼接 | **不同记录从不拼接** |
| 验证纪律 | 若共用偏置让某份验证变差，**保留并调查**，不把它并入训练后仍称独立验证 |

**历史验证结果 / Historical validation**：标签静止区间内的检出比例约
**100% / 100% / 63.33% / 93.31%**。

> 这只是**用户确认的静止片段内**的检出比例，**不是整体准确率**，
> 也不评价行走时的误触发率。重叠滑窗高度相关，不能当作独立实验次数。

---

## 6. 速度传播 / Velocity propagation

**实现**：`velocity_propagation.py` —— 世界系**单步**梯形积分。

```text
v_new = v_old + (0.5·a_old + 0.5·a_new) · Δt
```

| 设计 Design | 原因 Rationale |
|---|---|
| 输入为**已去重力**的世界系加速度 | 明确分工：本模块不做坐标变换 |
| `VelocityState` 为 `frozen=True` | 固化输入副本，外部列表改动不会污染状态 |
| `Δt ≤ 0` 抛错 | 重复或倒退的节点时间戳无意义 |
| `Δt > max_dt_s` 抛错 | **断流必须停止该段并显式重新初始化**，不跨未知间断积分 |
| 端点各取一半再相加 | 降低中间求和溢出风险 |
| 初值与外部修正**显式传入** | 修正必须在状态时刻、同一世界系给出，并保留该时刻加速度 |

> **本函数不做的事**：不执行 ZUPT、不做位置传播、不做延迟融合、
> 不传播协方差。
> It performs one integration step only — no ZUPT, no position, no covariance.

**诊断**：`validate_velocity_drift.py` 在静止平台上**假设起点速度为零**后自由
积分，输出末速度与峰值。**起点零速度是假设**，结果是漂移诊断，
不是轨迹，也不是独立动态精度验收。

**为什么误差会累积 / Why this matters**：平均仅偏 0.07 m/s² 的恒定偏差，
持续 10 s 就会带来约 **0.7 m/s** 速度误差与约 **3.5 m** 位置误差
（说明累积效应的数学例子，**不是**本实验已测得的位移）。

---

## 7. 尚未实现的算法 / Not implemented

| 未实现 | 说明 |
|---|---|
| 真正的 ZUPT 速度更新 | 目前只有候选许可信号；进入滤波器更新状态的部分未实现，归属待确认 |
| 位置传播 / 轨迹 | 无任何位置结果 |
| 完整 IMU 校准 | 温度漂移、刻度因数、精密安装角标定均未做（只有名义安装方向） |
| 加速度计偏置估计 | 去重力**不**估计加速度偏置 |
| 协方差 / 不确定性传播 | 速度传播不含协方差 |
| UWB 融合、ESKF、闭环 | 属于完整系统的其他部分，不在本仓库 |
| 磁力计辅助航向 | 传感器 x 轴 180° 二义性仍不可观测，需磁力计或 UWB 复核 |

---

## 8. 数值与健壮性约定 / Numerical robustness

| 约定 | 实现 |
|---|---|
| 拒绝而非猜测 | 非法输入抛 `ValueError`，不返回零或"安全值" |
| 排除 `bool` | 用 `type(x) is int` 而非 `isinstance`（`bool` 是 `int` 子类） |
| 有限性检查 | 所有进入算法的数值用 `math.isfinite` 检查 |
| 四元数归一化 | `quat_normalize()`，零四元数抛错 |
| 单位固定 | 内部全用 SI，禁止度/弧度混用 |
| 不可变状态 | `@dataclass(frozen=True)` + `object.__setattr__` 固化副本 |
| 时间由调用者提供 | 算法不读系统时钟 |

---

## 9. 结论的正确表述 / How to state the results

✅ 可以说 / Acceptable：

> 已完成真实 IMU 采集、姿态与去重力的初步验证，保留并分析了残差；
> 结果允许继续开发速度传播。

❌ 不可以说 / Not acceptable：

> "已经实现室内定位" / "ZUPT 已经做好" / "准确率 93.75%" /
> "转身误差 0.02°" / "六轴可以给出航向" / "大腿不动就是零速"。

依据与完整边界见 [docs/limitations.md](limitations.md) 与
[阶段评估报告](../analysis/imu_stage_review_20261014_0000/IMU阶段评估与详细记录.md)。
