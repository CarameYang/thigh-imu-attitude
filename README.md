# 大腿 IMU 姿态估计与去重力
# Thigh-Mounted IMU Attitude Estimation and Gravity Removal

从一块消费级六轴 IMU（ICM-20948）的真实硬件采集，到世界坐标系线加速度的
**完整、可回放、带明确边界**的本地实验链路。

A complete, replayable bench pipeline from a consumer six-axis IMU
(ICM-20948) to world-frame linear acceleration, with explicit scope limits.

**状态核对时间 / Status as of: 2026-10-14 14:55 (Asia/Shanghai, UTC+08:00)。**
本文件描述的是该时刻的真实实现状态，不是计划或期望状态。
This document describes the implementation that actually exists at that time,
not a planned or expected state.

> **一句话 / One line**
> 已完成：真实六轴采集 → 原始录制 → 数据与时间检查 → 静止候选检测 →
> 陀螺零偏 → 姿态估计 → 正式去重力 → 回放验证 → 速度单步传播。
> 未完成：真正的 ZUPT 更新、位置传播、轨迹精度验证、UWB 融合。
> Done: acquisition → raw recording → integrity/timing checks → stationary
> candidates → gyro bias → attitude → formal gravity removal → replay
> validation → one-step velocity propagation.
> Not done: actual ZUPT updates, position propagation, validated trajectory
> accuracy, UWB fusion.

---

## 目录 / Table of contents

- [1. 这是什么 / What this is](#1-这是什么--what-this-is)
- [2. 硬件与环境 / Hardware and environment](#2-硬件与环境--hardware-and-environment)
- [3. 数据流 / Data flow](#3-数据流--data-flow)
- [4. 快速开始 / Quick start](#4-快速开始--quick-start)
- [5. 仓库结构 / Repository layout](#5-仓库结构--repository-layout)
- [6. 文档地图 / Documentation map](#6-文档地图--documentation-map)
- [7. 核心约定速查 / Key conventions](#7-核心约定速查--key-conventions)
- [8. 结果与证据 / Results and evidence](#8-结果与证据--results-and-evidence)
- [9. 测试 / Tests](#9-测试--tests)
- [10. 数据与隐私 / Data and privacy](#10-数据与隐私--data-and-privacy)
- [11. 限制与常见误读 / Limitations](#11-限制与常见误读--limitations)
- [12. 路线图 / Roadmap](#12-路线图--roadmap)
- [13. 来源与致谢 / Provenance](#13-来源与致谢--provenance)
- [14. 许可 / License](#14-许可--license)
- [15. 发布前检查单 / Pre-publish checklist](#15-发布前检查单--pre-publish-checklist)

---

## 1. 这是什么 / What this is

这是一个**大腿佩戴 IMU 子模块**的独立实验与验证仓库。目标场景是可穿戴应急
人员的室内轨迹估计：GPS 不可用时，用惯性传感器在短时失去外部测距的情况下
维持状态估计。

This repository holds a standalone **thigh-mounted IMU sub-module** for indoor
trajectory estimation of emergency responders, where inertial sensing must
bridge short gaps without external ranging.

**重要**：本模块**不是**一个可用的定位系统。它当前做到的是"把真实 IMU 数据
可靠地采下来，算到世界系线加速度，并诚实量化还差多少"。
**Important**: this is **not** a working positioning system. It reliably
acquires real IMU data, computes world-frame linear acceleration, and honestly
quantifies the remaining error.

### 当前状态 / Current status

| 能力 Capability | 状态 Status | 证据 Evidence |
|---|---|---|
| 真实六轴采集（ESP32 → UDP → 电脑） | ✅ 已验证 | 真实录制 packets.jsonl，序号连续 |
| 原始数据保真（原始字节 + 解析值） | ✅ 已验证 | base64 原始包与解析结果一致 |
| 节点时间与接收时间分离 | ✅ 已验证 | 节点间隔 19.556–20.449 ms（约 50 Hz） |
| 陀螺零偏标定 | ✅ 已验证 | 按录制分别标定，不跨会话复用 |
| 姿态估计（Mahony 四元数互补滤波） | ✅ 已实现并测试 | 22 项姿态测试 |
| 去重力（世界系线加速度） | ✅ 已实现并测试 | 8 项测试 + 16 窗口真实回放 |
| 静止候选检测（`zupt_allowed`） | ✅ 已实现并测试 | 固定模型 + 11 项检测/训练测试 |
| 速度传播（世界系梯形积分） | 🟡 刚实现，未验证精度 | 8 项测试；只有静止漂移诊断 |
| 真正的 ZUPT 速度更新 | ❌ 未实现 | 候选 ≠ 已执行更新 |
| 位置传播 / 轨迹 | ❌ 未实现 | 无任何轨迹结果 |
| 动态精度与真值验证 | ❌ 未验证 | 无独立参考轨迹 |
| 完整 IMU 校准（温漂、刻度、安装角） | ❌ 未完成 | 只有名义安装方向 |
| UWB / 融合 / 实时 UI | ❌ 不属于本仓库 | 由完整系统的其他部分负责 |

### 设计原则 / Design principles

1. **原始数据不可覆盖。** 任何处理都不改写原始录制，处理结果另存。
   Raw data is never overwritten; processed results are stored separately.
2. **区分"测量时间"与"接收时间"。** 姿态积分只用节点时间差，不把电脑收包
   抖动当作运动。
   Node time drives integration; host arrival jitter is never treated as motion.
3. **区分"候选"与"更新"。** `zupt_allowed` 只是"允许考虑一次零速度观测"，
   不是速度已经归零。
   A candidate is not an update.
4. **不为了让曲线好看而删点或调参。** 尖峰、持续偏差与失败窗口全部保留。
   Spikes, persistent offsets and failing windows are retained, not tuned away.
5. **测试通过 ≠ 精度通过。** 单元测试证明计算行为，不证明真实运动精度。
   Passing tests prove computation, not real-world accuracy.

---

## 2. 硬件与环境 / Hardware and environment

| 项目 Item | 值 Value |
|---|---|
| MCU | DOIT ESP32 DEVKIT V1（ESP-WROOM-32，CP2102 USB 转串口） |
| IMU | SparkFun Qwiic 9DoF IMU **ICM-20948**（I²C，六轴使用） |
| 固件 Firmware | `imu_udp_50hz_test`（Arduino，非本仓库；约 50 Hz UDP 发送） |
| 端口 Port | UDP 9000（电脑接收），节点时间戳为整数微秒 |
| 节点名 Node ID | `esp32_lab_01`（`is_valid_imu` 会校验） |
| 安装方式 Mounting | **大腿**（thigh-mounted）；手持录制不用于判定安装方向 |
| 验证环境 Verified on | Python 3.14.6（`pyproject.toml` 声明 `requires-python >= 3.14`） |
| 依赖 Dependencies | **无第三方依赖**，仅 Python 标准库 / standard library only |

> **计划中的最终硬件**（本实验不代表已变更）：ICM-42688-P 足部/大腿 IMU、
> 4 × Qorvo DWM3000EVB + 4 × ESP32（3 锚点 + 1 标签）。
> The **planned final hardware** (unchanged by these bench experiments):
> ICM-42688-P and 4 × DWM3000EVB with 4 × ESP32.

固件单位换算（只读核对）：加速度库输出 mg × 0.00980665 → m/s²；
角速度 deg/s × 0.01745329252 → rad/s。固件保留原始三轴对应，不替换成世界坐标。
Firmware unit conversion (read-only verified): mg → m/s² and deg/s → rad/s;
raw sensor axes are preserved in transport.

---

## 3. 数据流 / Data flow

```text
        ┌──────────────────────── 采集侧 / Acquisition ────────────────────────┐
        │  ICM-20948 ──I²C──> ESP32 ──UDP/JSON, ~50 Hz──> 电脑 UDP 9000         │
        │                          (imu_udp_50hz_test 固件, 非本仓库)          │
        └───────────────────────────────┬──────────────────────────────────────┘
                                        │
                  ┌─────────────────────┴─────────────────────┐
                  │                                           │
        imu_recorder.py                            live_zupt_receiver.py
      （定时录制，保存原始字节）              （实时静止候选检测 + 保存状态）
                  │                                           │
                  └─────────────────────┬─────────────────────┘
                                        ▼
                    recordings/  ·  live_recordings/       ← 不入库 / not committed
                    packets.jsonl · states.jsonl · session.json · summary.json
                                        │
        ┌───────────────────────────────┴───────────────────────────────┐
        │                    离线回放与分析 / Offline replay             │
        │                                                               │
        │  check_pregravity.py      完整性、序号、节点时间间隔、姿态一致性 │
        │  attitude.py              陀螺零偏 + Mahony 四元数姿态 q_bw     │
        │  gravity_removal.py       a_world = R_bw·R_BS·f + (0,0,-9.80665)│
        │  stationary_stream.py     静止候选状态机（zupt_allowed）        │
        │  velocity_propagation.py  世界系单步梯形速度积分（无 ZUPT）      │
        │  validate_gravity_removal.py / validate_velocity_drift.py      │
        └───────────────────────────────┬───────────────────────────────┘
                                        ▼
                        analysis/<主题>_<时间戳>/    ← 文本报告入库
                                                      jsonl 逐窗输出不入库
```

---

## 4. 快速开始 / Quick start

### 4.1 环境 / Environment

```powershell
# 仅需 Python 3.14+，无需安装任何第三方包。
# Only Python 3.14+ is required; no third-party packages.
python --version
```

### 4.2 跑测试 / Run the tests

不依赖录制数据即可运行的 38 项（姿态 22 + 去重力 8 + 速度 8）：
38 tests run without any recording (22 attitude + 8 gravity + 8 velocity):

```powershell
python -m unittest test_attitude test_gravity_removal test_velocity_propagation
```

完整 49 项（另外 11 项需要本地 `recordings/` 与固定模型，见 §9）：
The full 49 (the other 11 need local `recordings/` and the fixed model):

```powershell
python -m unittest test_gravity_removal test_attitude test_stationary_stream `
                   test_zupt_offline test_train_zupt test_velocity_propagation
```

### 4.3 查看已有报告 / Read existing reports

本仓库的核心结果是**文字报告**，先读这两份：

| 报告 Report | 内容 Content |
|---|---|
| [IMU 阶段评估与详细记录](analysis/imu_stage_review_20261014_0000/IMU阶段评估与详细记录.md) | 方法、16 窗口完整结果表、三处残差分解、边界 |
| [去重力验证成果与复现指南](2026-10-14_1436_Gravity_Removal_Validation_Results_and_Reproduction_Guide.md) | 按阶段排列的复现命令、预期输出、展示顺序 |

### 4.4 用真实数据复现 / Reproduce with your own data

```powershell
# 1) 先录制（需要硬件；ESP32 上电并发送后开始计时）
python imu_recorder.py --label stationary --notes "IMU flat on desk"

# 2) 完整性、序号、节点时间间隔检查
python -X utf8 check_pregravity.py

# 3) 去重力 16 窗口统计（使用脚本内固定的四份录制）
python -X utf8 validate_gravity_removal.py

# 4) 六轴曲线查看
python -X utf8 visualize_imu.py --recordings live_recordings
```

> `-X utf8` 用于保证 Windows 终端正确显示中文输出。
> `-X utf8` keeps Chinese terminal output correct on Windows.

---

## 5. 仓库结构 / Repository layout

```text
.
├── README.md                     ← 本文件 / this file
├── LICENSE                       ← MIT
├── .gitignore
├── pyproject.toml                ← requires-python >= 3.14，无依赖 / no deps
│
├── docs/                         ← 本仓库的说明文档 / documentation (新增)
│   ├── architecture.md           模块职责与边界
│   ├── coordinates.md            坐标系、安装方向、四元数约定
│   ├── data-formats.md           UDP 包与录制文件字段
│   ├── algorithms.md             姿态 / 去重力 / 静止检测 / 速度传播
│   ├── experiments.md            录制清单、方法、证据规则
│   ├── limitations.md            限制、未完成项、常见误读
│   └── development.md            开发、测试、提交与数据政策
│
├── 采集与接收 / Acquisition
│   ├── imu_recorder.py           定时录制，保存原始字节与到达时间
│   ├── live_zupt_receiver.py     实时接收 + 静止候选 + 状态保存
│   ├── udp_rate_receiver.py      UDP 速率测量
│   ├── udp_test_receiver.py      最小 UDP 接收测试
│   └── update_imu_reconnect.py   录制收发工具
│
├── 算法模块 / Algorithm modules
│   ├── attitude.py               四元数 Mahony 互补滤波、R_BS、欧拉角
│   ├── gravity_removal.py        纯函数去重力（无网络/无滤波/无积分）
│   ├── stationary_stream.py      逐包静止候选状态机
│   ├── velocity_propagation.py   世界系单步梯形速度积分
│   ├── zupt_offline.py           离线零偏与候选检测（原单记录方案）
│   └── train_zupt.py             多记录训练/验证，输出固定模型
│
├── 检查与诊断 / Checks and diagnostics
│   ├── check_pregravity.py       完整性、时间、姿态内部一致性
│   ├── check_attitude_replay.py  四份录制姿态回放（含去重力）
│   ├── check_recording_direction.py  自动识别静止/转动段并判定方向
│   ├── verify_direction.py       绕垂直轴转身判定 +X 轴 180° 二义性
│   ├── diagnose_gyro_bias.py     标定窗口敏感性
│   ├── diagnose_zupt.py          固定录制的检测诊断图
│   ├── diagnose_live_zupt.py     实时录制段的独立诊断报告
│   ├── analyze_stationary.py     早期静止录制统计
│   └── visualize_imu.py          六轴交互查看器
│
├── 验证 / Validation
│   ├── validate_gravity_removal.py   16 窗口 mean/RMS/p95/peak
│   └── validate_velocity_drift.py    静止平台上的速度漂移诊断
│
├── 测试 / Tests（6 个模块，49 项 / 6 modules, 49 tests）
│   ├── test_attitude.py              22
│   ├── test_gravity_removal.py        8
│   ├── test_velocity_propagation.py   8
│   ├── test_stationary_stream.py      3   ← 需要录制数据
│   ├── test_zupt_offline.py           4   ← 需要录制数据
│   └── test_train_zupt.py             4   ← 需要录制数据
│
├── zupt_split.json               ← 用户确认的静止标签划分（训练/验证）
│
├── analysis/                     ← 分析输出目录（16 个），含可入库的报告
│   ├── imu_stage_review_20261014_0000/   阶段评估报告 + 窗口统计 + 文件哈希
│   ├── attitude_validation_20261012/     安装方向判定的依据
│   ├── mounting_check_20261012_003433/   佩戴安装检查
│   ├── gyro_check_20261012_004616/       陀螺符号与刻度核对
│   ├── zupt_training_20261007_164450_519158/  固定检测模型 model.json
│   └── …（其余为诊断与回放输出 / other diagnostics and replays）
│
└── 项目记录 / Project records
    ├── RECORDINGS_INDEX.md       录制动作清单
    └── ZUPT_TRAINING_README.md   检测器训练说明
```

**不入库 / Not committed**：`recordings/`、`live_recordings/`、`analysis/**/*.jsonl`、
`downloads/`、`repository-review/`、`.venv/`、`.idea/`、`.document_work/`、`*.docx`、
`personal.ipynb`，以及内部开发日志（`AGENTS.md`、`PROGRESS.md`、`NEXT_STEPS.md`）。
详见 [.gitignore](.gitignore)。

---

## 6. 文档地图 / Documentation map

| 想了解 / To learn about | 读这个 / Read |
|---|---|
| 每个模块做什么、边界在哪 | [docs/architecture.md](docs/architecture.md) |
| 坐标系、安装方向、四元数约定 | [docs/coordinates.md](docs/coordinates.md) |
| UDP 包与录制文件的字段 | [docs/data-formats.md](docs/data-formats.md) |
| 姿态、去重力、静止检测、速度传播算法 | [docs/algorithms.md](docs/algorithms.md) |
| 做过哪些实验、能证明什么 | [docs/experiments.md](docs/experiments.md) |
| 还差什么、哪些话不能说 | [docs/limitations.md](docs/limitations.md) |
| 环境、测试、提交与数据政策 | [docs/development.md](docs/development.md) |
| 真实结果与复现步骤 | [阶段评估报告](analysis/imu_stage_review_20261014_0000/IMU阶段评估与详细记录.md)、[复现指南](2026-10-14_1436_Gravity_Removal_Validation_Results_and_Reproduction_Guide.md) |
| 每份录制的实际动作 | [RECORDINGS_INDEX.md](RECORDINGS_INDEX.md) |
| 检测器训练细节 | [ZUPT_TRAINING_README.md](ZUPT_TRAINING_README.md) |

---

## 7. 核心约定速查 / Key conventions

| 约定 Convention | 值 Value |
|---|---|
| 传感器安装（名义） | **+X 向上、+Y 向左、+Z 向后**（丝印面朝腿外） |
| 身体坐标 / Body frame | +x 前、+y 左、+z 上（右手系） |
| 安装变换 / Mounting | `body = (−sensor Z, sensor Y, sensor X)`，即 `attitude.R_BS`，行列式 +1 |
| 世界坐标 / World frame | +z 向上；水平朝向在初始化时选定，**不是真北** |
| 四元数 / Quaternion | `(w, x, y, z)`，表示**身体到世界** `q_bw` |
| 单位 / Units | m、m/s、m/s²、rad、rad/s；重力常数 9.80665 m/s² |
| 时间戳 / Timestamps | 节点微秒整数 `timestamp_us`；**节点时钟与电脑时钟未同步** |
| 去重力公式 | `a_world = R_bw · R_BS · f_sensor + (0, 0, −9.80665)` |

> ⚠️ **历史资料中"传感器 +X → body −z"的写法已作废**，与当前定义和代码不符。
> Historical descriptions mapping sensor +X to body −z are **superseded**.
> 完整推导与依据见 [docs/coordinates.md](docs/coordinates.md)。

---

## 8. 结果与证据 / Results and evidence

基于 2026-10-12 的四份真实录制（合计 **13,478** 个有效包，节点间隔
19.556–20.449 ms，约 50 Hz），在**固定窗口**上回放去重力：

| 指标 Metric | 结果 Result | 边界 Caveat |
|---|---|---|
| 16 个评估窗口中三轴 RMS < 0.2 m/s² | **15 / 16** | 诊断窗口计数，**不是 93.75% 准确率**；0.2 不是正式验收门槛 |
| 手持 `025851` 8.5–10 s：X 均值 / RMS | −0.1814 / 0.2123 m/s² | 前后半段均超线，非单个尖峰造成 |
| 大腿 `003433` 31–39.5 s：Z 绝对峰值 | 1.6862 m/s² | 整体 RMS 仍较小，说明小 RMS 与尖峰并存 |
| 桌面 `004616` 45–59 s：X 绝对峰值 | 1.0920 m/s² | 相邻变化主要来自输入测量，非姿态跳变 |
| 慢转体估计航向变化 | ≈ 90.02° | 与约 90° 动作相符，但动作无精确真值，**不能称为 0.02° 误差** |
| 换标定窗口后的转身估计 | 90.02 / 87.48 / 84.44 / 79.59° | 结果对窗口敏感，不得挑最接近 90° 的当"最准" |
| 静止段残余航向变化 | 约 7.7° / 12.3° / 13.5° | 六轴无绝对航向，静止也会累积转动 |

全部 16 个窗口、全部样本均保留，未按检测结果筛选，未删点。
All 16 windows and all samples are retained; nothing was filtered or deleted.

**这些数字说明内部一致性，不说明动态精度。** 姿态估计复用同一个加速度计，
静止残差无法独立证明运动加速度或轨迹精度。
These numbers show internal consistency, not dynamic accuracy: attitude
estimation reuses the same accelerometer.

---

## 9. 测试 / Tests

最近一次联合运行（2026-10-14，本机）：**49 项全部通过**。
Last combined run (2026-10-14, local): **49 passed**.

| 测试模块 Module | 项数 | 是否需要录制数据 | 覆盖内容 Coverage |
|---|---:|---|---|
| `test_attitude.py` | 22 | 否 | 安装变换行列式、积分符号、四元数、物理回归、阶段就绪检查 |
| `test_gravity_removal.py` | 8 | 否 | 静止、倾斜/倒置、真实线加速度保留、转向、自由落体、时间与数值非法输入 |
| `test_velocity_propagation.py` | 8 | 否 | 梯形积分数学、时间倒退/重复、超时断流、显式初始化与外部修正 |
| `test_stationary_stream.py` | 3 | **是** | 状态机安全门与回放一致性 |
| `test_zupt_offline.py` | 4 | **是** | 离线检测关键行为 |
| `test_train_zupt.py` | 4 | **是** | 训练隔离与区间边界 |

> **克隆本仓库后 11 项会失败**，因为按项目规定原始录制不入库。
> 11 tests will fail in a fresh clone because recordings are intentionally
> not committed. 需要复现这 11 项时请自备 `recordings/` 与
> `analysis/zupt_training_20261007_164450_519158/model.json`。

---

## 10. 数据与隐私 / Data and privacy

- **原始实验录制不随本仓库发布。** 录制、逐包输出与回放产物均在
  [.gitignore](.gitignore) 中排除；本机保留，不因不提交而删除。
  Raw recordings are never published; they remain on the author's machine.
- **没有凭据。** 全量扫描未发现 API key、token、口令或私钥。
  No credentials are present in this repository.
- **不含任何可指向个人的信息。** 本机用户名与绝对路径、内网地址、姓名、所属
  机构及关联仓库地址均已清除，并由全库文本扫描确认。
  Nothing that points to an individual is published: host paths, LAN addresses,
  names, institution and related repositories were removed and verified by a
  full-text scan.
- **唯一的例外是固件协议常量。** 代码中的 `DIP_PING` / `DIP_ACK` 是与 ESP32
  固件约定的握手字节，改名会直接断开与硬件的通信，因此保留原样；它们是不透明
  的协议字面量，不指向任何个人或机构。
  The only retained token is the firmware handshake constant, which must match
  the device to keep the link working.

---

## 11. 限制与常见误读 / Limitations

**不要从本仓库得出以下结论 / Do not conclude:**

| ❌ 错误说法 | ✅ 实际情况 |
|---|---|
| "已经实现室内定位" | 只到世界系线加速度与单步速度积分，无位置、无轨迹 |
| "ZUPT 已经做好" | 只有静止**候选**；真正的零速度更新未实现 |
| "15/16 说明准确率 93.75%" | 那是诊断窗口计数，与定位准确率无关 |
| "转身误差只有 0.02°" | 动作本身没有精确真值，且结果对标定窗口敏感 |
| "大腿不动就是零速" | 大腿低动态 ≠ 脚落地零速 ≠ IMU 瞬时速度为零 |
| "六轴能给出航向" | 六轴无绝对航向参考，静止段也会累积转动 |
| "测试全过就代表精度达标" | 单元测试覆盖计算与输入保护，不覆盖真实运动精度 |
| "手持数据可用于判定佩戴方向" | 手持朝向由手决定，不能作为安装依据 |

完整的边界清单与物理原因分析见 [docs/limitations.md](docs/limitations.md)。

---

## 12. 路线图 / Roadmap

按项目既有顺序（未完成项，不代表已排期）：

1. **速度传播**（进行中）：模拟输入验证积分数学 → 用静止录制量化漂移；
   明确初始速度、有效性、断流处理与外部修正状态接口。
2. **位置传播**：短时走停回放；再针对缺失真值安排有实际距离的路线实验。
3. **真正的 ZUPT 更新**：与融合负责人确认时间、坐标、修正状态接口与分工。
4. **独立动态精度验证**：需要外部参考（测距路线、动作时间真值）。
5. **完整 IMU 校准**：温度漂移、刻度因数、精密安装角（当前只有名义方向）。
6. **交接与融合**：向系统集成方交付六轴数据、校准/安装说明、可接受外部修正的
   传播模块与静止候选材料。

---

## 13. 来源与范围 / Provenance and scope

本仓库是一个**可穿戴 IMU 子模块**的独立实验、实现与验证记录，聚焦大腿佩戴下
从六轴数据到世界系线加速度的链路。完整系统（UWB 测距、状态估计与融合、
后端与实时界面）不在本仓库范围内。
This repository is a standalone record of a **wearable IMU sub-module**: the
pipeline from thigh-mounted six-axis data to world-frame linear acceleration.
The wider system (UWB ranging, state estimation and fusion, backend and live
UI) is out of scope here.

与外部系统对接时，接口、坐标与数据包字段以对方契约为准；本仓库的 UDP 包字段
名与常见的 `version` / `node_id` / `sequence_number` / `timestamp_us` / `type` /
`data` 结构一致，但本地校验器刻意只覆盖本实验需要的字段。
When integrating with an external system, that system's contracts take
precedence. The UDP field names here follow the common
`version` / `node_id` / `sequence_number` / `timestamp_us` / `type` / `data`
layout, but the local validator intentionally checks only the fields this
experiment needs.

---

## 14. 许可 / License

[MIT](LICENSE) © 2026 CarameYang。

---

## 15. 发布前检查单 / Pre-publish checklist

**身份信息已处理**（2026-10-14 完成，逐文件扫描确认）：
Identity information has been removed and verified by a full-text scan:

- [x] 本机用户名与绝对路径（`C:\Users\<user>\…`）→ 占位符替换。
- [x] 内网地址（`192.168.x.x`、`172.x.x.x`）→ 占位符替换。
- [x] 姓名、所属机构、关联仓库地址 → 全部移除。
- [x] 内部开发日志（`AGENTS.md`、`PROGRESS.md`、`NEXT_STEPS.md`）→ 排除在
      [.gitignore](.gitignore) 之外，仅保留在本机。
- [x] 本地文档（`*.docx`）→ 不随仓库发布。
- [x] 保留的 `DIP_PING` / `DIP_ACK` 为固件握手常量，改名会断开硬件通信。

推送前仍需人工确认 / Still to confirm manually before pushing:

- [ ] **确认公开授权**：这部分内容与实测结果是否适合公开。
- [ ] **确认无大型文件**：`git status` 后检查是否有 > 50 MB 的文件
      （GitHub 单文件硬上限 100 MB）。
- [ ] **确认无凭据**：`git diff --cached` 通读一遍。
- [ ] **确认录制未入库**：`recordings/`、`live_recordings/`、`*.jsonl` 均被忽略。
- [ ] **确认内部日志未入库**：`AGENTS.md`、`PROGRESS.md`、`NEXT_STEPS.md` 不在
      待提交列表中。
- [ ] **更新状态日期**：修改 README 顶部与 §1 表格中的"状态核对时间"。

---

<div align="center">

**当前阶段的准确表述 / Accurate statement of this stage**

我们已经完成真实 IMU 数据采集、姿态与去重力的初步验证，保留并分析了残差。
当前结果允许继续开发速度传播，但**尚未得到通过独立真值验证的运动轨迹**。

We have completed real IMU acquisition and initial attitude and gravity-removal
validation, while retaining and analyzing residuals. These results support
continuing velocity-propagation development, but we do **not** yet have a
motion trajectory validated against independent ground truth.

</div>
