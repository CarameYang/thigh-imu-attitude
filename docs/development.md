# 开发指南 / Development Guide

面向修改本仓库的人（包括未来的自己）。

For anyone modifying this repository, including your future self.

---

## 1. 环境 / Environment

| 项目 | 要求 |
|---|---|
| Python | **3.14+**（`pyproject.toml` 声明 `requires-python >= "3.14"`；本机验证版本 3.14.6） |
| 第三方依赖 | **无**——全部为标准库 / standard library only |
| 操作系统 | 主要在 Windows + PowerShell 下开发；脚本本身不依赖 Windows 特性 |
| 终端编码 | 建议加 `-X utf8`，保证中文输出正确 |

```powershell
python --version          # 期望 3.14.x
python -m unittest --help # 确认标准库可用
```

> `pyproject.toml` 的 `dependencies` 为空是**有意为之**：采集、姿态、去重力、
> 检测与速度积分都不需要 numpy 或 scipy。新增依赖前请先说明为什么标准库不够。

---

## 2. 命令速查 / Command reference

下表列出每个脚本的用途与**是否写文件**，便于判断哪些命令是只读的。

| 命令 | 用途 | 需要硬件 | 写文件 |
|---|---|:---:|---|
| `python imu_recorder.py --label stationary --notes "…"` | 定时录制 60 s | ✅ | `recordings/<时间>_<label>/` |
| `python live_zupt_receiver.py --duration 90 --notes "…"` | 实时接收 + 静止候选 | ✅ | `live_recordings/<时间>_live/` |
| `python udp_rate_receiver.py` | UDP 速率/缺口测量 | ✅ | 否 |
| `python -X utf8 visualize_imu.py` | 六轴交互查看（早期录制） | ❌ | 否 |
| `python -X utf8 visualize_imu.py --recordings live_recordings` | 六轴交互查看（实时录制） | ❌ | 否 |
| `python -X utf8 analyze_stationary.py` | 早期静止统计 | ❌ | `analysis/stationary_stats.json`（覆盖） |
| `python -X utf8 diagnose_gyro_bias.py` | 零偏窗口敏感性 | ❌ | 否 |
| `python -X utf8 check_pregravity.py` | 完整性/时间/姿态一致性 | ❌ | 否 |
| `python -X utf8 check_attitude_replay.py` | 四份录制姿态回放（含去重力） | ❌ | 否 |
| `python -X utf8 validate_gravity_removal.py` | 16 窗口去重力统计 | ❌ | 否 |
| `python -X utf8 validate_velocity_drift.py` | 静止平台速度漂移 | ❌ | 否 |
| `python -X utf8 diagnose_zupt.py [--live] [--no-gui]` | 检测诊断图/报告 | ❌ | `analysis/zupt_diagnostic_<时间>/` |
| `python -X utf8 diagnose_live_zupt.py --no-gui` | 实时录制段独立诊断 | ❌ | `analysis/live_diagnostic_<时间>/` |
| `python -X utf8 train_zupt.py` | 重新训练检测器 | ❌ | `analysis/zupt_training_<时间>/` |
| `python -X utf8 train_zupt.py --model <模型> --session <录制>` | 复用模型 | ❌ | `analysis/zupt_training_<时间>/` |
| `python check_recording_direction.py <录制目录>` | 自动判定方向 | ❌ | 否 |
| `python verify_direction.py <packets.jsonl> --still1 a b --turn c d --still2 e f` | 转身判定 +X 二义性 | ❌ | 否 |
| `python update_imu_reconnect.py` | 录制收发/整理工具 | ❌ | 视参数而定 |

> ⚠️ `train_zupt.py` **不带 `--model`** 是**重新训练**，会生成新偏置与阈值，
> 不等于复用历史模型。实时接收器默认加载
> `analysis/zupt_training_20261007_164450_519158/model.json`。

---

## 3. 测试 / Tests

标准库 `unittest`，无需安装 pytest。当前共 **49 项**（2026-10-14 实测全通过）。

```powershell
# 全部 49 项（需要本地录制与固定模型）
python -m unittest test_gravity_removal test_attitude test_stationary_stream `
                   test_zupt_offline test_train_zupt test_velocity_propagation

# 仅 38 项自包含测试（克隆后即可运行）
python -m unittest test_attitude test_gravity_removal test_velocity_propagation
```

| 模块 | 项数 | 依赖录制数据 |
|---|---:|---|
| `test_attitude` | 22 | 否 |
| `test_gravity_removal` | 8 | 否 |
| `test_velocity_propagation` | 8 | 否 |
| `test_stationary_stream` | 3 | **是** |
| `test_zupt_offline` | 4 | **是** |
| `test_train_zupt` | 4 | **是** |

> 依赖数据的 11 项需要 `recordings/` 下的对应录制与固定 `model.json`。
> 若在本机缺少数据，请**如实说明未运行**，不要写成"测试通过"。
> If the data is missing, state that the tests were not run — never report a
> pass you did not observe.

---

## 4. 代码规范 / Coding conventions

### 4.1 注释必须中英双语 / Bilingual comments are required

本项目要求同一处注释同时给出中文和英文，且**含义一致**。

```python
# 先绑定端口，避免端口占用时创建空记录。
# Bind first to avoid creating empty sessions on port conflicts.
```

### 4.2 硬性约定 / Hard rules

| 规则 | 说明 |
|---|---|
| **不改写原始数据** | 处理结果另存；函数返回新对象 |
| **拒绝而不是猜测** | 非法输入抛 `ValueError`，**不返回零或默认值** |
| **排除 `bool`** | 用 `type(x) is int`（`bool` 是 `int` 子类） |
| **有限性检查** | 进入算法的数值一律 `math.isfinite` |
| **单位固定 SI** | m / m/s / m/s² / rad / rad/s；禁止度与弧度混用 |
| **时间由调用者给** | 算法不读系统时钟；时间戳为非负整数微秒 |
| **不可变状态** | `@dataclass(frozen=True)` + 固化输入副本 |
| **算法不碰网络/文件** | 纯模块只依赖标准库与更底层算法模块 |
| **依赖只向下** | 见 [docs/architecture.md](architecture.md) 的分层 |
| **docstring 写明边界** | 每个模块开头说明"做什么"与"**不做什么**" |

### 4.3 命名约定 / Naming

| 对象 | 约定 | 示例 |
|---|---|---|
| 模块 | 小写下划线 | `gravity_removal.py` |
| 测试模块 | `test_` + 被测模块 | `test_gravity_removal.py` |
| 录制目录 | `YYYYMMDD_HHMMSS_ffffff_<label>` | `20261012_003433_121271_live` |
| 分析目录 | `analysis/<主题>_<YYYYMMDD_HHMMSS_ffffff>/` | `analysis/zupt_training_20261007_164450_519158/` |
| 单位后缀 | 变量名带单位 | `acceleration_world_mps2`、`timestamp_us` |
| 坐标系后缀 | 变量名带参考系 | `q_bw`（body→world）、`f_sensor` |

---

## 5. 数据政策 / Data policy

| 规则 | 说明 |
|---|---|
| 原始录制**不入库** | `recordings/`、`live_recordings/`、`*.jsonl` 均在 `.gitignore` |
| 原始录制**不删除** | 不提交 ≠ 可以删；本机保留全部原始数据 |
| 每次录制**必须登记** | 在 [`RECORDINGS_INDEX.md`](../RECORDINGS_INDEX.md) 记录目录名、记录时间、实际动作、用途、能否作为安装依据 |
| 小型确定性样例可入库 | 放在 `data/samples/`（需先解除 `.gitignore` 中的例外注释） |
| 不提交大型日志/产物 | MCAP、bag、大型日志、安装包一律排除 |

### 新增一次录制的流程 / Adding a recording

1. 录制：`python live_zupt_receiver.py --duration <秒> --notes "<用途>"`
2. 记录 `session.json` 的 `created_at`（精确到分钟）。
3. 在 `RECORDINGS_INDEX.md` 写明：目录名、记录时间、**我给出的指令原文（含 `--notes`）**、
   **用户报告的实际动作**、用途、可否作为安装依据。
4. 动作秒数注明是**电脑接收相对时间**，并用自动识别的静止平台核对。
5. 只有**计划**而没有实际确认时，明确标注"计划，未独立确认"。

### 新增一次分析输出的流程 / Adding an analysis

1. 建立 `analysis/<主题>_<YYYYMMDD_HHMMSS_ffffff>/`。
2. 报告写 `.md`（中英对照），统计写 `.json`，摘要写 `.txt`，临时脚本写 `.py`。
3. 逐窗/逐包的大输出写 `.jsonl`（**不入库**，可重算）。
4. 若评估了某批文件，保存其 SHA-256 以便日后核对版本
   （参见 `reviewed_file_sha256.json`）。
5. 在项目进度记录中写明结论与依据；**不覆盖历史报告**。

---

## 6. Git 与协作 / Git workflow

| 规则 | 说明 |
|---|---|
| **不直接推 `main`** | 使用 `feature/…` 或 `fix/…` 分支 |
| **不强推共享分支** | 尤其是他人的分支 |
| 提交保持小而清晰 | 一次提交一件事，说明"改了什么 + 验证了什么" |
| 提交前检查差异 | 通读 `git diff --cached`，确认无凭据、无大文件、无调试残留 |
| 不合并自己的 PR 需授权 | 未经明确授权不合并 |
| 接口变更要标注 | 破坏性变更需同步更新生产端、消费端、示例、测试与文档 |
| 界面变更附截图 | 若涉及可视化脚本 |

### 推荐的首次提交顺序 / Suggested first commits

```powershell
git init
git add .gitignore LICENSE README.md docs/
git commit -m "docs: add README, license and documentation set"

git add *.py zupt_split.json pyproject.toml
git commit -m "feat: add IMU attitude, gravity-removal and velocity modules with tests"

git add analysis/imu_stage_review_20261014_0000 analysis/attitude_validation_20261012
git commit -m "docs: add stage assessment report and validation evidence"

# 提交前务必确认：git status 中没有 recordings/、live_recordings/、*.jsonl、*.docx
```

---

## 7. 故障排查 / Troubleshooting

| 现象 | 可能原因 | 处理 |
|---|---|---|
| 中文输出乱码 | 终端编码 | 用 `python -X utf8 …` |
| `无法监听 / Cannot bind UDP 9000` | 另一个接收器在运行 | 关闭其它接收器；`live_zupt_receiver.py` 会先绑定再建目录，不会留下空录制 |
| `No valid IMU data within 30 seconds` | 节点未上电/未连 Wi-Fi/端口不符 | 先跑 `udp_test_receiver.py` 验证链路 |
| 实时状态一直是 `unknown` | 未站稳、缺包、或模型路径不对 | 看 `reason` 字段；确认 `--model` 指向存在的 `model.json` |
| 测试报缺少文件 | 11 项依赖本地录制 | 自备 `recordings/`；或只跑 38 项自包含测试 |
| 静止时线加速度接近 ±19.6 m/s² | 四元数方向写反（`q_bw` ↔ `q_wb`） | 见 [docs/coordinates.md](coordinates.md) §3 |
| 重力残差随姿态变化很大 | 忘记应用 `R_BS` | 见 [docs/coordinates.md](coordinates.md) §2 |
| 速度出现不合理尖峰 | 用了电脑接收时间差，或未处理断流 | 积分只用节点时间；`max_dt_s` 触发时必须停止该段 |
| 结果与历史报告不一致 | 模型/参数/数据版本不同 | 用 `reviewed_file_sha256.json` 核对版本，记录差异 |

---

## 8. 起步阅读顺序 / Where to start reading

1. [README](../README.md) —— 状态与边界。
2. [docs/architecture.md](architecture.md) —— 哪个文件干什么。
3. [docs/coordinates.md](coordinates.md) —— **最容易出错的地方**。
4. [docs/data-formats.md](data-formats.md) —— 数据结构。
5. [docs/algorithms.md](algorithms.md) —— 算法与参数。
6. [docs/limitations.md](limitations.md) —— 不能说的话。
7. `analysis/imu_stage_review_20261014_0000/IMU阶段评估与详细记录.md` —— 真实结果。
