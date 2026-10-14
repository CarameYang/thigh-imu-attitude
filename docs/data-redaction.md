# 录制数据脱敏 / Recording Redaction

本仓库**发布真实实验录制**，但内容是脱敏副本。本文说明删除了什么、为什么、
以及如何自行复核或重新生成。
This repository **publishes the real experiment recordings** in redacted form.
This document states what was removed, why, and how to verify or regenerate it.

---

## 1. 为什么需要脱敏 / Why redaction is necessary

录制文件由 `imu_recorder.py` 与 `live_zupt_receiver.py` 写入。每一行除了测量数据
之外，还包含**接收端补充的字段**：
Recordings contain **receiver-added fields** alongside the measurements:

```json
{"arrival_unix_ns": 1760200473121271000,
 "arrival_monotonic_ns": 616130728676700,
 "sender_ip": "10.61.52.110",
 "sender_port": 9001,
 "valid_imu": true,
 "raw_base64": "eyJ2ZXJzaW9uIjogMSwg...",
 "packet": {"version": 1, "node_id": "esp32_lab_01", "...": "..."},
 "accepted_for_detection": true}
```

其中两项与隐私直接相关：
Two of these are privacy-relevant:

| 字段 | 问题 |
|---|---|
| `sender_ip` / `sender_port` | 记录采集时 ESP32 在局域网中的地址。全部录制中共出现 **50,278** 次，涉及两个私有地址。 |
| `arrival_unix_ns` | 电脑墙上时钟的纳秒时间戳，逐包记录。 |

`raw_base64` 不含网络信息（UDP 头不在载荷内），它是解析结果的冗余副本，
删除它是为了把数据体积降到约 60%。
`raw_base64` carries no network information — it is a redundant copy of the
parsed payload, removed only to cut the published size to about 60%.

---

## 2. 删除了什么，保留了什么 / What changed

运行 `python tools/redact_recordings.py --source _raw_backup --target .` 后：

| 处理 | 字段 |
|---|---|
| **删除** | `sender_ip`、`sender_port`、`arrival_unix_ns`、`raw_base64` |
| **保留** | `arrival_monotonic_ns`、`valid_imu`、`packet`、`accepted_for_detection` |

**测量数值本身完全未改动。** 逐条比对确认：记录条数一致、`packet` 对象逐字段
一致、`arrival_monotonic_ns` 一致。因此所有分析结果不受影响。
**Measurements are unchanged.** Record counts, the `packet` objects and
`arrival_monotonic_ns` were verified element-by-element, so every analysis
result is unaffected.

体积变化 / Size:

| 目录 | 原始 | 脱敏后 |
|---|---:|---:|
| `recordings/` | 12.43 MB | 5.61 MB |
| `live_recordings/` | 37.13 MB | 24.01 MB |
| 合计 | 49.55 MB | **29.62 MB** |

`states.jsonl`、`session.json`、`summary.json` 与录制目录内的 `model.json`
均逐字复制，只对 `model.json` 的 `sources` 做了两项处理：
Everything else is copied verbatim; only `model.json` needed two changes:

1. `sources[].path` 中的本机绝对路径 → `<repo-root>/<根目录>/<会话名>`
2. `sources[].sha256` 按**脱敏后**的文件重算，使已发布的模型与已发布的数据自洽

---

## 3. 没有删除什么 / What was deliberately kept

**时间戳没有改。** 这一点需要说明清楚：
**Timestamps were not altered.** This needs an explicit explanation:

- 各录制的日期与时刻（如 `2026-10-12 00:34`）**本来就已经公开**：目录名本身就是
  时间戳，且 [`RECORDINGS_INDEX.md`](../RECORDINGS_INDEX.md) 与两份分析报告都
  逐条列出了记录时间与实际动作。
- 只在 `session.json` 里改写 `created_at` 会与上述文档产生矛盾，而且**起不到隐藏
  作用**——同样的信息仍可从目录名和清单读到。

> 如果目标是隐藏"什么时候做了什么"，需要连带重命名 18 个录制目录、并更新
> `RECORDINGS_INDEX.md`、`zupt_split.json`、代码默认路径与两份分析报告。
> 这是另一项独立且影响面更大的改动，**本次没有执行**。

If the goal were to hide the experiment timeline, it would require renaming all
18 session directories and updating every cross-reference. That is a separate,
much larger change and was **not** performed.

---

## 4. 如何复核 / How to verify

```powershell
# 1) 确认已发布的录制里没有网络字段
Select-String -Path recordings\*\packets.jsonl,live_recordings\*\packets.jsonl `
              -Pattern 'sender_ip|raw_base64|arrival_unix_ns'

# 2) 确认模型里记录的哈希与已发布文件一致
python -c "import hashlib,json,pathlib;[print(n, hashlib.sha256((pathlib.Path('recordings')/n/'packets.jsonl').read_bytes()).hexdigest() == e['sha256']) for p in pathlib.Path('analysis').rglob('model.json') for n,e in (json.loads(p.read_text(encoding='utf-8')).get('sources') or {}).items() if (pathlib.Path('recordings')/n).is_dir()]"

# 3) 重新生成脱敏副本（需要本机的 _raw_backup/）
python tools/redact_recordings.py --source _raw_backup --target .
```

`check_pregravity.py` 在缺少 `raw_base64` 时会**明确打印跳过原因**，而不是静默通过：
When `raw_base64` is absent, `check_pregravity.py` **prints the reason it skipped**
that check rather than passing silently:

```text
20261012_003433_121271_live: raw bytes not published for 4190 of 4190 records; raw-vs-parsed check skipped
```

---

## 5. 这对复现意味着什么 / Effect on reproducibility

| 可以复现 | 不能复现 |
|---|---|
| 全部去重力窗口统计（16 窗口 mean/RMS/p95/peak） | 原始字节与解析结果的逐条一致性核对 |
| 姿态回放、静止候选检测、检测器训练与验证 | 从原始 UDP 字节重新解析 |
| 六轴曲线、完整性、序号与节点时间间隔检查 | — |
| 45 项不需要录制的单元测试 + 依赖录制的 4 项 | — |

`analysis/imu_stage_review_20261014_0000/reviewed_file_sha256.json` 记录的是
**评估当时**（脱敏之前）的文件哈希，因此其中四个录制的哈希与已发布文件不再一致。
该文件保留原值，作为"当时评估了哪些文件"的历史凭据，不随脱敏重写。
The hashes in `reviewed_file_sha256.json` describe the files as they were **at
assessment time**, before redaction, and are intentionally left unchanged.

---

## 6. 本机原始数据 / Local originals

未脱敏的原始录制保存在本机 `_raw_backup/`（已被 `.gitignore` 排除，不随仓库发布）。
The untouched originals live in the local `_raw_backup/` directory, which is
excluded from the repository.

**新增录制后必须重新运行脱敏工具**，否则新录制会以未脱敏状态被提交：
Every new recording must be redacted before it is committed:

```powershell
# 采集完成后 / after recording
python tools/redact_recordings.py --source _raw_backup --target .   # 先同步 _raw_backup
git status                                                          # 确认无 sender_ip 字段后提交
```
