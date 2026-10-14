# 数据格式 / Data Formats

本文给出**实际写入磁盘和网络上的字段**，字段名与类型以当前代码为准。
This document lists the fields actually written to the wire and to disk.

来源代码 / Source of truth：`imu_recorder.py`、`live_zupt_receiver.py`、
`stationary_stream.py`、`train_zupt.py`、`zupt_split.json`。

---

## 1. 数据分三层 / Three layers

```text
① 线路格式      UDP JSON 数据报（约 50 Hz，端口 9000）
                    ↓  imu_recorder.py / live_zupt_receiver.py
② 录制文件      packets.jsonl（逐包）、session.json（条件）、summary.json（概览）
                              states.jsonl（仅实时接收器）
                    ↓  离线脚本
③ 分析输出      analysis/<主题>_<时间戳>/ 下的 json / txt / jsonl / md
```

**① 与 ② 的区别**：线路格式是固件发出的原始包；录制文件在其外层再包一层
接收元数据（到达时间、发送方、原始字节）。**原始字节始终保留**，因此解析器
的改动可以事后复核。
The raw datagram is always preserved in `raw_base64`, so parser changes can be
re-checked after the fact.

---

## 2. 线路格式：UDP JSON 包 / Wire format

固件 `imu_udp_50hz_test` 发出，单播到电脑 UDP **9000**：

```json
{
  "version": 1,
  "node_id": "esp32_lab_01",
  "sequence_number": 327,
  "timestamp_us": 6652000,
  "type": "imu",
  "data": {
    "ax_mps2": 0.12,  "ay_mps2": -0.34, "az_mps2": 9.79,
    "gx_radps": 0.001, "gy_radps": -0.002, "gz_radps": 0.0005
  }
}
```

| 字段 Field | 类型 Type | 含义 Meaning |
|---|---|---|
| `version` | int | 协议版本，本实验固定 `1` |
| `node_id` | string | 节点名，本实验固定 `esp32_lab_01` |
| `sequence_number` | 非负 int | 节点侧递增序号，用于缺口/重复/乱序检查 |
| `timestamp_us` | 非负 int | **节点**时钟的整数微秒；非硬件采样时间戳 |
| `type` | string | 固定 `imu` |
| `data.ax/ay/az_mps2` | finite number | 三轴加速度（比力），m/s² |
| `data.gx/gy/gz_radps` | finite number | 三轴角速度，rad/s |

### 校验规则 / Validation (`imu_recorder.is_valid_imu`)

拒绝条件（任一不满足即判为无效，包仍会被保存但标记 `valid_imu: false`）：

1. 顶层不是 JSON 对象；
2. `version != 1`、`node_id != "esp32_lab_01"` 或 `type != "imu"`；
3. `sequence_number` / `timestamp_us` 不是非负整数
   （用 `type(x) is int` 判断，**排除 `bool`**，因为 `bool` 是 `int` 的子类）；
4. `data` 不是对象，或六个字段中任一缺失、非数值、为 `NaN` / `±inf`。

> **这是本实验所需的窄校验，不代替生产端完整协议验证。**
> This is a deliberately narrow check for the bench experiment.

### 探测与应答 / Probe and ACK

除 IMU 包外还有一条简单链路探测：

```text
电脑 → 节点 :  b"DIP_PING " + 16 位十六进制 token
节点 → 电脑 :  b"DIP_ACK "  + 同一 token
```

接收器回复 ACK 并把探测包与 IMU 包分开计数，**不把探测包当作 IMU 数据**。

---

## 3. 录制目录 / Recording directories

命名 / Naming：`<YYYYMMDD_HHMMSS_ffffff>_<label>`

| 接收器 Receiver | 目录 Root | 后缀 Suffix | 时长控制 |
|---|---|---|---|
| `imu_recorder.py` | `recordings/` | `_stationary` 或 `_motion` | 固定 **60 s**，从首个有效包到达起算；无包则 30 s 后退出 |
| `live_zupt_receiver.py` | `live_recordings/` | `_live` | `--duration` 秒（0 = 不限，Ctrl+C 停止） |

每次运行建立**唯一目录**（`exist_ok=False`），不覆盖历史实验。

| 文件 File | `imu_recorder` | `live_zupt_receiver` |
|---|:---:|:---:|
| `packets.jsonl` | ✅ | ✅ |
| `session.json` | ✅ | ✅ |
| `summary.json` | ✅ | ✅（概览计数） |
| `states.jsonl` | — | ✅ |
| `model.json` | — | ✅（所用检测模型的副本） |

---

## 4. `packets.jsonl` —— 逐包记录 / Per-packet records

一行一个完整 JSON 对象（JSON Lines），便于流式读取与中断后保留已写数据。
One complete JSON object per line.

| 字段 Field | 类型 | 说明 |
|---|---|---|
| `arrival_unix_ns` | int | 电脑墙上时钟（纳秒），仅记录 |
| `arrival_monotonic_ns` | int | 电脑单调时钟（纳秒），用于时长与超时判断 |
| `sender_ip` | string | UDP 源地址 |
| `sender_port` | int | UDP 源端口 |
| `valid_imu` | bool | 是否通过 §2 校验 |
| `raw_base64` | string | **精确的原始字节**（含无效包），用于事后复核解析 |
| `packet` | object \| null | 解析后的包；仅 `valid_imu = true` 时写入，否则为 `null` |

`live_zupt_receiver.py` 额外写入 / Additional field in the live receiver：

| 字段 | 类型 | 说明 |
|---|---|---|
| `accepted_for_detection` | bool | `valid_imu` 且发送方未变（非外来发送者）时才进入检测 |

> **外来发送者保护**：实时接收器锁定第一个发送方 `(ip, port)`，之后来自其他
> 地址的包不会被送入检测状态机（计入 `foreign_sender`）。
> The live receiver locks onto the first sender and ignores others.

---

## 5. `session.json` —— 会话条件 / Session conditions

`imu_recorder.py`：

```json
{
  "created_at": "2026-10-12T00:34:33.121271+08:00",
  "label": "motion",
  "notes": "Thigh mounting + slow 90deg body rotation validation",
  "duration_target_s": 60.0,
  "node_id": "esp32_lab_01",
  "hardware": "DOIT ESP32 DEVKIT V1 + SparkFun ICM-20948",
  "timestamp_note": "Host receive timestamps are recorded after recvfrom; node and host clocks are not synchronized."
}
```

`live_zupt_receiver.py`：

```json
{
  "created_at": "…+08:00",
  "mounting": "thigh",
  "notes": "…",
  "duration_s": 0,
  "model_sha256": "…",
  "confirmation_s": 0.5,
  "receive_timeout_s": 0.5,
  "timestamp_note": "MCU and host clocks are not synchronized. States are candidates, not displacement measurements."
}
```

> `model_sha256` 把该次实时判断**绑定到具体模型版本**：模型换了，历史结果不会被
> 误当作同一套判据。
> The model hash pins each live session to the exact detector model used.

> ⚠️ **历史元数据注意**：早期某次录制（`20261007_155937_947511_motion`）的
> `session.json` 里保留着 `Foot-mounted` 的说明，但用户已确认该次及后续录制为
> **大腿佩戴**。按项目规则，**保留原始元数据不改写**，解释时以用户确认为准。
> Original metadata is preserved as-is; the user's confirmation governs
> interpretation.

---

## 6. `summary.json` —— 快速概览 / Quick overview

| 字段 | 说明 |
|---|---|
| `ended_at` | 结束时间（本地时区 ISO 8601） |
| `reason` | 结束原因：`completed` / `no_valid_imu_within_30s` / `duration_complete` / `stopped` 等 |
| `valid_imu_packets` | 有效 IMU 包数 |
| `invalid_or_other_packets` | 无效或非 IMU 包数 |
| `ack_send_failures` | ACK 发送失败次数 |
| `arrival_rate_hz` | 平均到达速率（基于电脑接收时间） |
| `first_node_timestamp_us` / `last_node_timestamp_us` | 节点时间戳范围 |
| `node_timestamp_nonincreasing_count` | 节点时间未严格递增的次数 |

> `summary.json` 只是概览，**不替代**对序号缺口、重复、乱序和逐包间隔的完整
> 离线分析（后者由 `check_pregravity.py` 与 `udp_rate_receiver.py` 承担）。

---

## 7. `states.jsonl` —— 实时静止候选状态 / Live detector states

`live_zupt_receiver.py` 写入。公共字段 + 状态机字段直接展开在同一行：

| 字段 | 类型 | 说明 |
|---|---|---|
| `event` | string | 触发事件：`packet` / `timeout` / `invalid_packet` / `stopped` … |
| `arrival_monotonic_ns` | int | 事件时刻（电脑单调时钟） |
| `elapsed_s` | float \| null | 相对首包的电脑接收相对秒数 |
| `state` | string | `unknown` / `confirming` / `stationary_candidate` / `not_stationary_candidate` |
| `zupt_allowed` | bool | **仅在确认后为 true**；表示"允许考虑一次零速度观测" |
| `reason` | string | 见下表 |
| `features` | array \| null | `[gyro_rms_radps, accel_vector_rms_mps2, gravity_error_rms_mps2]` |
| `candidate_duration_s` | float | 仅在 `confirming` / `stationary_candidate` 时出现 |

**`reason` 取值 / Reasons：**

| 值 | 触发条件 |
|---|---|
| `warming_up` | 缓冲尚未覆盖完整时间窗 |
| `invalid_measurement` | 时间戳/序号/数值非法（同时重置状态机） |
| `gap_or_reset` | 序号不连续、时间倒退，或相邻间隔 > `max_gap_s`（默认 0.04 s） |
| `threshold_exceeded` | 特征超出阈值 |
| `awaiting_confirmation` | 特征达标但持续时长不足（`confirming`） |
| `sustained_low_dynamics` | 持续低动态已确认（`stationary_candidate`） |
| `no_fresh_data` | 0.5 s 无新有效包，必须撤销候选状态 |

---

## 8. `model.json` —— 固定检测模型 / Fixed detector model

`train_zupt.py` 训练输出，`stationary_stream.py` 与 `live_zupt_receiver.py` 消费。

| 键 Key | 说明 |
|---|---|
| `schema_version` | 模型格式版本 |
| `kind` | 模型类型标识 |
| `bias_radps` | 合并后的陀螺零偏（各训练区间等权平均，rad/s） |
| `thresholds` | 三个特征的阈值（最大区间 99% 分位 × 1.5，并取配置下限） |
| `detector` | `window_s`（0.2）、`max_gap_s`（0.04）等时序参数 |
| `training` | 训练区间统计与依据 |
| `training_bias_range_radps` | 各训练区间偏置的范围（说明偏置并非完全相同） |
| `bias_policy` / `threshold_policy` | 明确记录"固定偏置""不因验证差而放宽"等策略 |
| `limitations` | 模型自带的使用限制说明 |
| `created_at` | 训练时间 |
| `sources` | 源录制与文件哈希 |

---

## 9. `zupt_split.json` —— 训练/验证划分 / Labeled split

```json
{
  "schema_version": 1,
  "label_source": "user_confirmed_stationary",
  "latest_session_mounting": "thigh",
  "training": [
    {"session": "20260923_024502_927703_stationary", "start_s": 0, "end_s": null},
    {"session": "20260923_025602_464673_stationary", "start_s": 0, "end_s": null},
    {"session": "20261007_155937_947511_motion",     "start_s": 55, "end_s": 60}
  ],
  "validation": [
    {"session": "20260923_025833_233431_motion", "start_s": 5, "end_s": 10},
    {"session": "20260923_162429_405653_motion", "start_s": 5, "end_s": 10},
    {"session": "20261007_155209_788135_motion", "start_s": 5, "end_s": 10},
    {"session": "20261007_155937_947511_motion", "start_s": 5, "end_s": 10}
  ],
  "detector": {
    "window_s": 0.2,
    "max_gap_s": 0.04,
    "training_quantile": 0.99,
    "threshold_multiplier": 1.5,
    "threshold_floors": [0.06, 0.15, 0.2]
  }
}
```

时间语义 / Time semantics：

- `null` 的 `end_s` 表示"到该录制结束"。
- 区间为**半开区间** `[start_s, end_s)`，相对各自的**首个节点样本**，末端不含。
- **训练段与验证段严格分离**，禁止把验证段并入训练后仍声称独立验证。

---

## 10. 分析输出 / Analysis outputs

命名 / Naming：`analysis/<主题>_<YYYYMMDD_HHMMSS_ffffff>/`

| 文件类型 | 入库 | 说明 |
|---|:---:|---|
| `.md` | ✅ | 报告（如阶段评估、安装判定依据） |
| `.json` | ✅ | 结构化统计（如 16 窗口指标、文件哈希） |
| `.txt` | ✅ | 双语结果摘要 |
| `.py` | ✅ | 该次分析使用的临时脚本 |
| `.jsonl` | ❌ | 逐窗/逐包输出（体积大、可由代码重算） |
| `.pyc` | ❌ | Python 缓存 |

已入库的关键产物 / Committed key artifacts：

| 路径 | 内容 |
|---|---|
| `analysis/imu_stage_review_20261014_0000/IMU阶段评估与详细记录.md` | 完整方法与 16 窗口结果表 |
| `analysis/imu_stage_review_20261014_0000/gravity_window_metrics.json` | 全部窗口 mean / RMS / p95 / peak |
| `analysis/imu_stage_review_20261014_0000/reviewed_file_sha256.json` | 评估时程序与录制的 SHA-256 |
| `analysis/zupt_training_20261007_164450_519158/model.json` | 当前实时接收器默认加载的固定模型 |

> **注意**：`live_zupt_receiver.py` 与 `diagnose_zupt.py` 默认加载上述固定模型
> 路径。若删除该目录，需要相应修改 `MODEL` 常量或使用 `--model` 指定新模型。

---

## 11. 已核实的完整性事实 / Verified integrity facts

基于 2026-10-12 的四份录制（详见 `docs/experiments.md`）：

| 项目 | 结果 |
|---|---|
| 有效包总数 | 13,478 |
| 序号连续性 | 连续，未发现缺口 |
| 节点时间 | 严格递增 |
| 原始字节 vs 解析结果 | 一致 |
| 非有限数值 | 未发现 |
| 相邻整组六轴完全相同 | 未发现（**只说明未发现整组重复**，不代表每个轴都有全新采样） |
| 节点相邻间隔 | 约 19.556–20.449 ms（约 50 Hz） |
| 电脑相邻到达间隔 | 有抖动，曾出现约 221 ms |

**仍未验证 / Still unverified**：传感器内部 ODR、精确采样时刻、端到端延迟、
跨节点同步、长期稳定性。
