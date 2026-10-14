# 软件架构与模块边界 / Architecture and Module Boundaries

本文说明本仓库的模块分层、依赖方向与设计规则。目的是让新读者能在不读完全部
代码的情况下判断"某个问题该看哪个文件"。

This document describes layering, dependency direction and design rules, so a
new reader can tell which file owns which problem without reading all the code.

---

## 1. 分层 / Layering

```text
第 3 层 验证与诊断 / Validation & diagnostics
        validate_gravity_removal.py   validate_velocity_drift.py
        check_pregravity.py           diagnose_gyro_bias.py
        diagnose_zupt.py              diagnose_live_zupt.py
        check_recording_direction.py  verify_direction.py
                        │
                        ▼  仅调用下层，不反向被依赖
第 2 层 回放与状态机 / Replay & state machines
        check_attitude_replay.py      stationary_stream.py
        live_zupt_receiver.py         train_zupt.py
                        │
                        ▼
第 1 层 纯算法 / Pure algorithm modules
        attitude.py   gravity_removal.py   velocity_propagation.py
        zupt_offline.py
                        │
                        ▼
第 0 层 采集与工具 / Acquisition & utilities
        imu_recorder.py   udp_rate_receiver.py   udp_test_receiver.py
        visualize_imu.py  analyze_stationary.py  update_imu_reconnect.py
```

**依赖只能向下。** 算法层不 import 采集层或验证层；验证层可以 import 任何下层。
纯算法模块不打开套接字、不读写录制目录。
**Dependencies point downward only.** Pure algorithm modules never open sockets
or touch recording directories.

实测依赖关系 / Verified import graph：

```text
attitude            ← gravity_removal ← (validate_gravity_removal, test_gravity_removal,
                        │                 test_velocity_propagation)
                        └← check_attitude_replay ← (check_pregravity, diagnose_gyro_bias,
                        │                            validate_gravity_removal,
                        │                            validate_velocity_drift)
velocity_propagation ← (validate_velocity_drift, test_velocity_propagation)

zupt_offline        ← train_zupt ← (stationary_stream, diagnose_zupt,
                        │            diagnose_live_zupt, test_train_zupt,
                        │            test_stationary_stream)
                        └← stationary_stream ← live_zupt_receiver

imu_recorder        ← live_zupt_receiver
```

注意两点 / Two notable consequences:

- `gravity_removal` 依赖 `attitude`（需要四元数类型与约定），但 `attitude`
  不依赖 `gravity_removal`——不存在循环。
- **在线与离线共用同一套特征与阈值代码**：`stationary_stream`（实时状态机）
  与 `diagnose_zupt`（离线诊断）都经由 `train_zupt` / `zupt_offline`。
  这保证了实时判断与离线复算使用同一判据。

---

## 2. 模块清单 / Module inventory

### 第 0 层：采集与工具 / Acquisition and utilities

| 文件 | 职责 | 接触网络 | 写文件 |
|---|---|---|---|
| `imu_recorder.py` | UDP 接收、固定 60 s 定时录制、保存**原始字节**（base64）、到达时间、`session.json` / `summary.json`；同时提供 `is_valid_imu()` 作为全仓库的包头校验 | 是 | 是（`recordings/`） |
| `live_zupt_receiver.py` | 实时接收 + 静止候选状态机 + 逐包状态保存（`packets.jsonl` / `states.jsonl`）；启动时复制所用 `model.json` 以固定版本 | 是 | 是（`live_recordings/`） |
| `udp_rate_receiver.py` | 测量 UDP 到达速率与缺号/重复/乱序 | 是 | 否 |
| `udp_test_receiver.py` | 最小可用 UDP 接收测试（链路连通性） | 是 | 否 |
| `update_imu_reconnect.py` | 录制收发/整理工具 | 否 | 是 |
| `visualize_imu.py` | 纯标准库六轴交互查看器（标准库绘图，无第三方依赖） | 否 | 否 |
| `analyze_stationary.py` | 早期两份静止录制统计，写 `analysis/stationary_stats.json` | 否 | 是 |

> **`imu_recorder.py` 的固定行为**：`--label` 必填（`stationary` / `motion`），
> 时长固定 **60 s**，从**首个有效 IMU 包到达**开始计时；每次建立唯一目录
> `recordings/YYYYMMDD_HHMMSS_ffffff_<label>`，不覆盖历史实验。

### 第 1 层：纯算法 / Pure algorithm modules

| 文件 | 职责 | 关键接口 |
|---|---|---|
| `attitude.py` | 陀螺零偏标定、四元数运算、Mahony 互补滤波姿态估计、安装矩阵 `R_BS` | `AttitudeEstimator.align_to_gravity()` / `.update()`、`body_from_sensor()`、`euler_from_quat()` |
| `gravity_removal.py` | 去重力纯函数：传感器比力 → 世界系线加速度 | `remove_gravity(f_sensor, q_bw, *, timestamp_us, ...)` → `GravityRemovedSample` |
| `velocity_propagation.py` | 世界系**单步**梯形速度积分；不执行 ZUPT、不做位置传播 | `VelocityState`、`propagate_velocity(previous, *, timestamp_us, acceleration_world_mps2, max_dt_s)` |
| `zupt_offline.py` | 原单记录方案：偏置估计、0.2 s 滑窗特征、候选判定 | `load()`、`windows()`、`features()`、`analyze()` |

**纯算法的共同约束 / Shared constraints：**

1. **不改写输入**：函数返回新对象，不修改传入的序列或字典。
2. **拒绝而不是猜测**：非法输入抛 `ValueError`，**不返回看似安全的零向量**。
3. **时间由调用者提供**：算法不读系统时钟。
4. **单位固定为 SI**：m / m/s / m/s² / rad / rad/s。

### 第 2 层：回放与状态机 / Replay and state machines

| 文件 | 职责 | 备注 |
|---|---|---|
| `check_attitude_replay.py` | 四份固定录制的姿态回放，并调用正式去重力模块；对外暴露 `SESSIONS` 与 `replay()` | 被验证层三个脚本复用 |
| `stationary_stream.py` | 逐包静止候选状态机：0.2 s 窗口、0.5 s 持续确认、缺包/坏值/超时撤销许可 | `push()` 返回 `state` / `zupt_allowed` / `reason` / `features` |
| `train_zupt.py` | 多记录训练与验证，等权合并各区间偏置，输出固定模型与逐窗检测 | 训练与验证区间严格分离 |
| `live_zupt_receiver.py` | 见第 0 层；同时是"实时链路"的集成点 | 复用 `imu_recorder.is_valid_imu` |

### 第 3 层：验证与诊断 / Validation and diagnostics

| 文件 | 回答什么问题 | 输出 |
|---|---|---|
| `validate_gravity_removal.py` | 去重力在 16 个固定窗口上剩多少误差 | 终端表格（mean / RMS / p95 / peak） |
| `validate_velocity_drift.py` | 静止平台上自由积分会漂多少 | 终端表格（末速、峰值、节点时长） |
| `check_pregravity.py` | 数据是否完整、序号是否连续、节点时间是否合理、姿态是否内部一致 | 终端检查结果 |
| `diagnose_gyro_bias.py` | 标定窗口的选择对转身估计影响多大 | 终端对比 |
| `diagnose_zupt.py` | 固定模型为什么把某窗判为静止候选 | `analysis/zupt_diagnostic_<时间>/` |
| `diagnose_live_zupt.py` | 某段实时录制的独立诊断 | `analysis/live_diagnostic_<时间>/` |
| `check_recording_direction.py` | 自动找静止段与转动段并判定方向 | 终端判定与依据 |
| `verify_direction.py` | 绕垂直轴转身可判定传感器 +X 的 180° 二义性 | 终端判定 |

---

## 3. 设计规则 / Design rules

| 规则 Rule | 原因 Rationale |
|---|---|
| 原始数据只读，处理结果另存 | 让改进后的算法可以重算同一份实验，历史结论可追溯 |
| 区分节点时间与电脑接收时间 | 电脑收包会抖动（曾出现 221 ms 间隔），不能当作运动 |
| 算法模块不碰网络与文件 | 便于单元测试，避免采集问题与积分问题互相掩盖 |
| 在线与离线共用特征代码 | 实时判断必须能被离线复算，否则无法解释 |
| 错误即停止该段，不跳过坏点继续积分 | 跨未知间断积分会产生无意义的速度与位置 |
| 候选状态与滤波更新分离 | `zupt_allowed` 只是许可信号；真正的零速度更新属于融合层 |
| 每次运行生成带时间戳的新目录 | 不覆盖历史报告，两版结果不会被混为一谈 |

---

## 4. 扩展点 / Extension points

新增一个算法模块（以已经落地的 `velocity_propagation.py` 为例）：

1. 建纯模块，只依赖标准库与更底层的算法模块，写明中英双语 docstring 与边界。
2. 定义**不可变**状态类型（`@dataclass(frozen=True)`），固化输入副本。
3. 明确失败条件并抛 `ValueError`：时间倒退、重复时间戳、超时断流、非有限值。
4. 用**已知输入的解析解**写单元测试（如恒定加速度 `v = a·t`）。
5. 用已有录制跑一遍诊断脚本，把结果写入 `analysis/<主题>_<时间戳>/`。
6. 若引入新的录制，按项目规定在 `RECORDINGS_INDEX.md` 登记实际动作。

> 已验证的边界：`propagate_velocity()` 只做**一步**传播，初始值与外部修正必须
> 由调用者通过显式 `VelocityState` 传入；延迟修正需要调用者自行回放，本函数
> 不实现延迟融合或协方差传播。
> `propagate_velocity()` performs one step only; initialization and external
> corrections are passed explicitly, and delayed fusion is out of scope.

---

## 5. 与团队主线仓库的边界 / Boundary with the team repository

本仓库**只负责 IMU 子模块**。以下内容不属于本仓库，差异时以团队主线仓库的
`contracts/` 为准：

| 归属 | 内容 |
|---|---|
| 本仓库 | 六轴采集、时间与完整性检查、姿态、去重力、静止候选、速度传播 |
| 主线仓库 | UWB 测距与锚点、ESKF/融合、后端服务、WebSocket、Web UI、MCAP 记录与回放 |
| 待确认 | 真正的 ZUPT 更新归属、修正状态接口、时间/坐标对齐方式 |

**接口提醒**：本仓库的 UDP 包头（`version` / `node_id` / `sequence_number` /
`timestamp_us` / `type` / `data`）与主线仓库契约**字段名一致**，但本仓库的
`is_valid_imu()` 只校验本实验所需字段（固定 `node_id = esp32_lab_01`、
`version = 1`），**不代替生产端完整协议验证**。
Field names match the team contract, but the local validator is intentionally
narrow and is not a full production schema check.
