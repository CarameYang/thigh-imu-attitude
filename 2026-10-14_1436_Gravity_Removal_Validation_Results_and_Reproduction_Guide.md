# 去重力验证完成阶段：成果展示与复现指南 / Gravity-Removal Validation Stage: Results and Reproduction Guide

记录时间：2026-10-14 14:36（Asia/Shanghai，UTC+08:00）。这是整理文档的时间，不是各次实验发生时间。

目的：保存刚才对话中各操作阶段的成果展示方式、复现命令、预期结果和解释边界，方便自己复习或向他人展示。

依据：`RECORDINGS_INDEX.md`、`ZUPT_TRAINING_README.md`，以及相关程序的命令参数。旧记录与最新交接冲突时，以最新交接及当前实现为准。本次只整理指南，没有重新执行实验或测试；下文“已通过”的数字来自此前已记录的运行结果。

**English**

Recorded: 2026-10-14 14:36 (Asia/Shanghai, UTC+08:00). This is the document preparation time, not the time of each experiment. The original timestamp is retained for this bilingual revision.

Purpose: preserve the presentation methods, reproduction commands, expected results and interpretation limits discussed for each stage, for personal review and team demonstrations.

Sources: `RECORDINGS_INDEX.md`, `ZUPT_TRAINING_README.md`, and the relevant program arguments. When older records conflict with the latest handoff, use the latest handoff and current implementation. This document update did not rerun experiments or tests; historical pass counts below come from previously recorded runs.

中英按节对应；命令仅列一次，两种语言共用。 / Chinese and English are paired by section; command blocks are shared and appear only once.

## 1. 当前完成到哪里 / Current status

已经有真实六轴采集、原始录制、数据与时间检查、静止候选检测、安装轴向说明、陀螺零偏处理、姿态估计、正式去重力及录制回放验证。残差中的持续偏差与两处尖峰也完成了初步数值来源分析。

当前结论是**阶段性、有条件通过，可以进入受控速度传播开发**。这里的“去重力验证完成”指已完成当前软件测试、固定录制回放和初步残差检查，不代表动态加速度或定位精度正式验收通过。

尚无真实速度/位置积分结果、真正的 ZUPT 速度修正结果或融合轨迹。因此现在应展示六轴曲线、静止候选、姿态回放与去重力统计，不能把这些称作“已经实现定位”。

### English — Current status

Real six-axis acquisition, raw recording, data and timing checks, stationary candidate detection, mounting-axis documentation, gyro bias handling, attitude estimation, gravity removal and recorded-data replay are available. Initial numerical analysis of a persistent residual and two spikes is also complete.

The conclusion is a **conditional engineering-stage pass: controlled velocity-propagation development may begin**. “Gravity-removal validation complete” means that the current software tests, fixed-recording replay and initial residual checks are complete. It does not mean formal acceptance of dynamic acceleration or positioning accuracy.

There are no actual velocity/position integration results, actual ZUPT velocity corrections or fused trajectories yet. Present six-axis plots, stationary candidates, attitude replay and gravity-removal statistics without claiming that positioning has been implemented.

## 2. 运行前准备 / Before running commands

在 PowerShell 中先运行：

```powershell
$repoRoot = "<repo-root>"
$py = "python"
Set-Location $repoRoot
```

下面的命令均在同一个终端使用上述变量。已有录制的回放不需要连接 ESP32，也不需要重新佩戴或录制。图形窗口打开后，可以先关闭窗口再继续下一项。

“展示”是打开已有图表或报告；“复现”是使用保存的数据和当前程序重新计算。复现旧数据不等于重新完成一次独立硬件实验。

### English — Before running commands

Run the PowerShell setup block above first. All subsequent commands use those variables in the same terminal. Replaying saved recordings requires neither a connected ESP32 nor another wearing/recording session. Close a plot window before continuing to the next command if needed.

“Display” means opening existing plots or reports. “Reproduce” means recalculating from saved data using the current programs. Replaying old data is not a new independent hardware experiment. The command blocks are shared by both language versions and should be run only once.

## 3. 阶段一：展示真实六轴数据 / Stage 1: Display real six-axis data

### 打开较早的录制

```powershell
& $py -X utf8 "$repoRoot\visualize_imu.py"
```

### 打开后来的实时录制文件

```powershell
& $py -X utf8 "$repoRoot\visualize_imu.py" --recordings "$repoRoot\live_recordings"
```

在窗口中选择录制，查看三轴加速度和三轴角速度随时间变化。可以对照 `RECORDINGS_INDEX.md` 中的实际动作，找到站立、转身、翻转或行走的时间段。

展示时可以说：“我们已保存真实 IMU 六轴数据，能够离线重复查看，而不是只看一次实时画面。”

注意：窗口中的平滑曲线只是帮助观察；陀螺减均值预览也不等于正式标定。加速度计原始读数包含静止时的重力相关比力，静止时不应要求三个轴都为零。

### English — Stage 1: Display real six-axis data

The first command opens the earlier recordings; the second opens files in `live_recordings`. Select a recording to inspect the three acceleration and three angular-rate traces over time. Compare the actual actions in `RECORDINGS_INDEX.md` to locate standing, turning, flipping or walking intervals.

Suggested explanation: “We have saved real six-axis IMU data that can be reviewed repeatedly offline, rather than relying on a single live display.”

Smoothed curves are viewing aids only. The gyro mean-subtraction preview is not formal calibration. Raw accelerometer readings include gravity-related specific force at rest, so all three raw axes should not be expected to read zero.

## 4. 阶段二：静止数据统计与零偏检查 / Stage 2: Stationary statistics and gyro bias checks

### 早期静止数据统计

```powershell
& $py -X utf8 "$repoRoot\analyze_stationary.py"
```

程序使用预设的两份旧静止录制，并写入或覆盖 `analysis/stationary_stats.json`。如果只想查看历史结果，直接打开已有文件即可。

这一阶段观察：模块不动时角速度是否仍有固定偏移、读数有多少波动、加速度模长是否接近重力大小。静止陀螺均值可以帮助估计零偏，但不能假定所有时间、温度和录制都共用完全相同的偏置。

### 当前姿态链路的零偏敏感性诊断

```powershell
& $py -X utf8 "$repoRoot\diagnose_gyro_bias.py"
```

查看不同窗口的陀螺均值、波动，以及换用不同标定窗口后转身角和后续漂移如何变化。此前记录中，转身结果随标定窗口变化约为 90.02°、87.48°、84.44°、79.59°。这说明标定窗口会影响结果，不能挑最接近 90° 的窗口就宣称精度最高。

用户已确认大腿录制 `20261012_003433_121271_live` 的电脑接收相对 2–6 秒身体与模块保持不动。必须保留这项事实，不能因读数存在波动而改写动作标签。

### English — Stage 2: Stationary statistics and gyro bias checks

**Earlier stationary statistics.** `analyze_stationary.py` uses two predefined older stationary recordings and writes or overwrites `analysis/stationary_stats.json`. Open the existing file directly if you only want to view the historical result.

Check whether angular rate has a persistent offset while the module is still, how much readings fluctuate, and whether the acceleration magnitude is close to gravity. The stationary gyro mean helps estimate bias, but identical bias cannot be assumed across all times, temperatures and recordings.

**Bias sensitivity in the current attitude pipeline.** `diagnose_gyro_bias.py` shows gyro means and variation across windows, and how calibration-window selection changes the estimated turn and subsequent drift. Previously recorded turn estimates were approximately 90.02°, 87.48°, 84.44° and 79.59° for different calibration windows. Selecting the result closest to 90° does not establish the best accuracy.

The user confirmed that both body and module remained still during host receive-relative seconds 2–6 of thigh recording `20261012_003433_121271_live`. Preserve this fact; fluctuations in the readings do not justify relabeling the action.

## 5. 阶段三：静止候选检测 / Stage 3: Stationary candidate detection

### 查看已保存成果

固定模型及历史结果位于：

```text
analysis/zupt_training_20261007_164450_519158/
```

主要文件：

| 文件 | 用途 |
| --- | --- |
| `model.json` | 保存偏置、阈值及训练依据 |
| `split.json` | 记录训练与验证区间 |
| `summary.txt` | 阅读结果摘要 |
| `validation.json` | 查看各录制的详细验证统计 |
| `*_detections.jsonl` | 查看逐窗口特征和候选状态 |

### 图形展示

```powershell
& $py -X utf8 "$repoRoot\diagnose_zupt.py"
```

显示固定录制的诊断图，并产生新的 `analysis/zupt_diagnostic_<时间>/` 报告目录。可以比较曲线、阈值和检测状态，解释程序为什么把某些窗口判为静止候选。

### 复用固定模型重新计算

```powershell
& $py -X utf8 "$repoRoot\train_zupt.py" --model "$repoRoot\analysis\zupt_training_20261007_164450_519158\model.json" --session "$repoRoot\recordings\20261007_155209_788135_motion"
```

这会应用已保存的模型并产生新的结果目录，不重新拟合偏置或阈值。若直接运行不带 `--model` 的训练程序，含义是重新训练，不是复用旧模型。

历史静止验证区间的检出比例约为 100%、100%、63.33%、93.31%。这些只是对应静止标签内的检出比例，不是全场景准确率，也不能据此评价行走时的误触发率。

特别注意：IMU 安装在大腿。大腿低动态不等于脚落地零速；“静止候选”也不等于已经执行了 ZUPT。检测用的共用偏置与姿态回放按录制估计的零偏，不是同一套标定结果。

### English — Stage 3: Stationary candidate detection

**Saved results.** The fixed model and historical results are in `analysis/zupt_training_20261007_164450_519158/`.

| File | Purpose |
| --- | --- |
| `model.json` | Saved bias, thresholds and training evidence |
| `split.json` | Training and validation intervals |
| `summary.txt` | Results summary |
| `validation.json` | Detailed per-recording validation statistics |
| `*_detections.jsonl` | Per-window features and candidate states |

**Plot display.** `diagnose_zupt.py` displays diagnostics for a fixed recording and creates a new `analysis/zupt_diagnostic_<timestamp>/` report directory. Compare traces, thresholds and detection states to explain why particular windows become stationary candidates.

**Recalculate using the fixed model.** The command with `--model` and `--session` applies the saved model and creates a new results directory without refitting bias or thresholds. Running the training program without `--model` performs training instead of reusing the saved model.

Historical detection fractions within the labeled stationary validation intervals were approximately 100%, 100%, 63.33% and 93.31%. They are not overall accuracy and do not measure false triggers during walking.

The IMU is thigh-mounted. Low thigh dynamics do not establish zero velocity at foot contact, and a stationary candidate does not mean a ZUPT update has been performed. The shared detector bias and the per-recording bias used for attitude replay are different calibration results.

## 6. 阶段四：展示已保存的实时检测诊断 / Stage 4: Review saved live-detection diagnostics

```powershell
& $py -X utf8 "$repoRoot\diagnose_zupt.py" --live
```

此命令读取旧录制 `live_recordings/20261011_034935_192932_live`，展示约 33–36 秒的诊断曲线。这里的 `--live` 指查看实时实验保存下来的数据，不是启动一次新采集。

该录制约 34.5 秒的动作已由用户澄清为“身体小幅晃动，双脚未换位置”。不能将其描述为已经证实的静止漏检：双脚未移动并不保证大腿 IMU 瞬时速度为零。

如需重新生成该段独立诊断报告，可运行：

```powershell
& $py -X utf8 "$repoRoot\diagnose_live_zupt.py" --no-gui
```

报告写入新的 `analysis/live_diagnostic_<时间>/`。当前展示任务无需运行实时接收器或新增录制。

### English — Stage 4: Review saved live-detection diagnostics

`diagnose_zupt.py --live` reads the saved recording `live_recordings/20261011_034935_192932_live` and displays diagnostics around seconds 33–36. Here, `--live` refers to data saved from a live experiment; it does not start acquisition.

The user clarified that the event near 34.5 seconds was slight body sway without relocating the feet. It is not a confirmed stationary-detection miss: unmoved feet do not guarantee zero instantaneous velocity of a thigh IMU.

To regenerate a separate diagnostic report for that interval, use the `diagnose_live_zupt.py --no-gui` command above. It writes a new `analysis/live_diagnostic_<timestamp>/` directory. This presentation task does not require starting the live receiver or making a new recording.

## 7. 阶段五：数据完整性、时间与姿态回放 / Stage 5: Data integrity, timing and attitude replay

### 数据完整性检查

```powershell
& $py -X utf8 "$repoRoot\check_pregravity.py"
```

检查四份固定录制的原始与解析数据、序号、节点时间间隔、姿态数值及静止平台表现。此前共检查 13,478 个样本，节点相邻间隔约 19.556–20.449 ms；序号连续，原始数据与解析保存内容一致。

这支持“保存的数据可以用于当前回放”。它不证明传感器内部采样率、跨节点同步或长期稳定性已经验证。电脑收包有抖动，因此姿态积分使用节点时间差，不能直接用电脑到包时间差代替。

### 姿态回放

```powershell
& $py -X utf8 "$repoRoot\check_attitude_replay.py"
```

查看固定录制的姿态相关终端结果。当前脚本也会调用正式去重力模块；它不是单独的三维姿态动画。

采用当前约定：传感器 +X 向上、+Y 向左、+Z 向后；身体坐标取前、左、上，对应 `body=(-sensor Z, sensor Y, sensor X)`。这是名义安装变换，不是精密安装角标定。历史说明中与此相反的 X 映射不再采用。

大腿慢转身的估计变化约 90.02°，与用户约 90° 的动作描述相符，但动作没有精确角度真值，不能宣称误差只有 0.02°。六轴系统仍有航向漂移，静止段也能出现明显累计转角。

不要用旧姿态分析脚本中的历史结论替代当前回放结果，或覆盖旧报告后将两版结果混为一谈。

### English — Stage 5: Data integrity, timing and attitude replay

**Data integrity.** `check_pregravity.py` checks raw versus parsed data, sequence numbers, node time intervals, attitude values and stationary plateaus in four fixed recordings. The previous check covered 13,478 samples. Adjacent node intervals were approximately 19.556–20.449 ms; sequences were continuous and raw data agreed with parsed saved values.

This supports using these recordings for the current replay. It does not establish the sensor's internal sampling rate, synchronization between nodes or long-term stability. Host packet arrival times fluctuate, so attitude integration uses node time differences rather than host arrival intervals.

**Attitude replay.** `check_attitude_replay.py` prints attitude-related results for the fixed recordings and also calls the formal gravity-removal module. It is not a separate 3D attitude animation.

Current convention: sensor +X up, +Y left, +Z back; body axes forward, left, up. Thus `body=(-sensor Z, sensor Y, sensor X)`. This is a nominal mounting transform, not precision mounting-angle calibration. Historical descriptions with the opposite X mapping are superseded.

The estimated slow thigh turn was about 90.02°, consistent with the user's approximate 90° action. Without precise reference angles, this cannot be called a 0.02° error. Six-axis heading still drifts, including appreciable accumulated rotation during stationary intervals.

Do not substitute conclusions from old attitude-analysis scripts for current replay results or overwrite old reports and conflate the two versions.

## 8. 阶段六：正式去重力结果 / Stage 6: Gravity-removal results

```powershell
& $py -X utf8 "$repoRoot\validate_gravity_removal.py"
```

程序使用四份 2026-10-12 录制，共 16 个固定评估窗口。先将传感器比力转到世界坐标，再计算：

```text
a_world = R_bw × R_BS × f_sensor + (0, 0, -9.80665)
```

直观理解：先弄清楚传感器朝向，再扣除静止时重力对应的比力，得到世界坐标下的线加速度。不能不管朝向就给某一个原始轴减 9.80665。

输出解释：

| 字段 | 学生版解释 |
| --- | --- |
| `calibration` | 用于初始化与标定的区间，与评估段分开 |
| `n` | 当前评估窗口的样本数 |
| `mean` | 有正负方向的平均残差，持续偏离零提示偏差 |
| `RMS` | 综合反映偏差和波动的大小，越小通常越接近零 |
| `abs_p95` | 约 95% 样本的绝对值不超过这一水平 |
| `abs_peak` | 绝对值最大的样本，帮助发现瞬时尖峰 |
| `below_reference` | 三轴 RMS 是否都小于诊断参考线 0.2 m/s² |

这些加速度统计均按世界 X、Y、Z 顺序，单位 m/s²。输出窗口秒数是电脑接收相对时间，0 为首包；计算姿态的时间差来自节点时间。这与早期训练标签按首个节点样本计时的约定不同。

此前可复现结果：

- 16 个窗口中 15 个满足每轴 RMS < 0.2；这是诊断窗口计数，不是 93.75% 准确率，0.2 也不是已约定的正式验收门槛。
- 手持 `025851` 的 8.5–10 秒窗口，X 均值约 -0.1814、RMS 约 0.2123。前后半段 X RMS 均超过参考线，说明不是仅由单个尖峰造成。
- 大腿 `003433` 的 31–39.5 秒窗口，Z 绝对峰值约 1.6862；桌面 `004616` 的 45–59 秒窗口，X 绝对峰值约 1.0920。整体 RMS 较小仍可能有瞬时尖峰。

全部窗口和样本都保留，没有按静止检测颜色筛选，也没有为了让结果好看而删点。姿态估计使用同一加速度计，静止残差主要说明内部一致性，不能单独证明运动加速度或轨迹精度。

### English — Stage 6: Gravity-removal results

`validate_gravity_removal.py` uses four recordings from 2026-10-12 and 16 fixed evaluation windows. It rotates sensor specific force into world coordinates, then applies the formula shown above.

In simple terms, first determine the sensor orientation, then subtract the gravity-related stationary specific force to obtain world-frame linear acceleration. Subtracting 9.80665 from an arbitrary raw axis without considering orientation is incorrect.

| Field | Plain-language meaning |
| --- | --- |
| `calibration` | Initialization/calibration interval, separate from evaluation |
| `n` | Number of samples in the evaluation window |
| `mean` | Signed average residual; persistent offset suggests bias |
| `RMS` | Combined size of bias and fluctuations; smaller usually means closer to zero |
| `abs_p95` | Approximately 95% of absolute sample values are at or below this level |
| `abs_peak` | Largest absolute sample value, useful for finding spikes |
| `below_reference` | Whether all three axis RMS values are below the diagnostic 0.2 m/s² line |

Acceleration statistics are ordered world X, Y, Z in m/s². Reported window times are relative to the first packet received by the host; attitude time differences come from node timestamps. This differs from the earlier training-label convention measured from the first node sample.

Previously reproduced results:

- 15 of 16 windows have per-axis RMS < 0.2. This is a diagnostic window count, not 93.75% accuracy; 0.2 is not an agreed formal acceptance threshold.
- In handheld recording `025851`, seconds 8.5–10 have X mean about -0.1814 and RMS about 0.2123. Both half-window X RMS values exceed the line, so a single isolated spike does not explain the result.
- In thigh recording `003433`, seconds 31–39.5 have a Z absolute peak of about 1.6862. In tabletop recording `004616`, seconds 45–59 have an X absolute peak of about 1.0920. A small overall RMS can coexist with spikes.

All windows and samples are retained, without filtering by detector color or deleting points to improve the result. Attitude estimation reuses the same accelerometer, so stationary residuals mainly check internal consistency; they do not independently prove dynamic acceleration or trajectory accuracy.

## 9. 阶段七：残差原因分析如何展示 / Stage 7: Present the residual analysis

打开详细学生版记录：

[IMU阶段评估与详细记录](analysis/imu_stage_review_20261014_0000/IMU阶段评估与详细记录.md)

同目录还有：

- `gravity_window_metrics.json`：全部 16 个窗口的详细统计。
- `reviewed_file_sha256.json`：评估时有关程序和录制文件的哈希，用于核对文件版本。

可以讲清楚三个发现：

1. 手持窗口约 93% 的 X 残差为负，具有持续偏差。数学分解中，方向项对 X 均值贡献约 -0.17969，幅值项约 -0.00174 m/s²，不能只靠调整重力常数解决。
2. 大腿约 35.028 秒的 Z 尖峰，相邻变化主要来自输入测量变化，而不是姿态突然跳变。
3. 桌面约 52.134 秒的 X 尖峰同样主要由输入测量变化贡献。

这些是数值贡献的定位，不是已经确定物理原因。不能直接断言用户动了、传感器坏了或网络丢包了。

**复现范围说明：**上述残差贡献分解当时用临时分析计算完成，结论已记录，但尚未保存成独立一键诊断脚本。现有去重力回放可重新得到窗口统计；要完整重算每项分解，需要另行整理分析程序。不要把“有详细报告”说成“所有分析已有一键复现工具”。

### English — Stage 7: Present the residual analysis

Open the detailed student-oriented assessment linked above. The same directory contains:

- `gravity_window_metrics.json`: detailed statistics for all 16 windows.
- `reviewed_file_sha256.json`: hashes of the reviewed programs and recordings, used to check file versions.

Three findings can be presented:

1. About 93% of handheld-window X residuals are negative, indicating a persistent offset. In the mathematical decomposition, the direction term contributes about -0.17969 and the magnitude term about -0.00174 m/s² to the X mean. Changing only the gravity constant cannot resolve this.
2. Near 35.028 seconds in the thigh recording, the adjacent Z spike change is mainly caused by changes in input measurements rather than an abrupt attitude change.
3. Near 52.134 seconds in the tabletop recording, the adjacent X spike change is also dominated by input measurements.

These identify numerical contributions, not unique physical causes. They do not establish that the user moved, the sensor failed or the network lost packets.

**Reproduction scope:** the residual-contribution decomposition was performed using temporary analysis calculations. Its conclusions are recorded, but it has not been saved as an independent one-command diagnostic script. Existing replay reproduces window statistics; reproducing every decomposition requires preparing a separate analysis program. A detailed report does not mean every analysis already has a one-command reproduction tool.

## 10. 阶段八：软件测试证据 / Stage 8: Software test evidence

```powershell
& $py -m unittest test_gravity_removal test_attitude test_stationary_stream test_zupt_offline test_train_zupt
```

最近记录的一次联合运行是 41 项全部通过：姿态 22 项、去重力 8 项、检测与训练相关 11 项。文件和环境未改变时，预期终端显示 `Ran 41 tests` 及 `OK`；如果本次实际运行失败，应保留错误，不能套用历史通过结论。

测试主要证明已覆盖案例中的计算和输入保护符合预期，例如坐标旋转、偏置扣除、真实线加速度保留，以及拒绝无效数据。测试通过不等于真实人体运动下的定位精度通过。

### English — Stage 8: Software test evidence

The most recently recorded combined run passed all 41 tests: 22 attitude tests, 8 gravity-removal tests and 11 detection/training tests. With unchanged files and environment, the expected terminal result is `Ran 41 tests` and `OK`. If a new run fails, retain the error; historical success cannot replace the current result.

Tests establish expected calculations and input guards for covered cases, including coordinate rotation, bias subtraction, preservation of real linear acceleration and rejection of invalid data. Passing software tests does not establish positioning accuracy during real human motion.

## 11. 推荐展示顺序 / Suggested presentation order

1. 六轴图：说明数据来自真实硬件，并能离线重放。
2. 静止候选图：说明如何判断“可能静止”，同时解释大腿安装限制。
3. 数据检查与姿态回放：说明时间、轴向与姿态计算已有检查，也仍有漂移。
4. 去重力终端统计：展示 15/16 窗口、超参考线窗口及尖峰，不只挑最好结果。
5. 残差报告与测试结果：说明已经分析到哪里、哪些物理原因未知，以及软件测试范围。
6. 结束时说明下一步是速度传播开发与静止漂移量化，之后才是位置传播和有参考距离的独立实测。

可使用的阶段说明：“我们已经完成真实 IMU 数据采集、姿态和去重力的初步验证，保留并分析了残差。当前结果允许继续开发速度传播，但尚未得到通过独立真值验证的运动轨迹。”

### English — Suggested presentation order

1. Six-axis plots: show that the data comes from real hardware and can be replayed offline.
2. Stationary-candidate plots: explain how “possibly stationary” is detected and the limitations of thigh mounting.
3. Data checks and attitude replay: explain the timing, axis and attitude checks, while retaining the drift limitations.
4. Gravity-removal terminal statistics: show the 15/16 count, the window above the reference line and the spikes, rather than selecting only favorable results.
5. Residual report and test results: explain the completed analysis, unknown physical causes and software-test coverage.
6. Explain that the next step is velocity-propagation development and stationary-drift measurement, followed by position propagation and independent trials with reference distances.

Suggested stage statement: “We have completed real IMU acquisition and initial attitude and gravity-removal validation, while retaining and analyzing residuals. The results support proceeding with velocity-propagation development, but we do not yet have a motion trajectory validated against independent ground truth.”

## 12. 操作影响速查 / Quick reference: operational effects

| 操作 | 是否需要硬件 | 文件影响 |
| --- | --- | --- |
| 六轴图、`diagnose_zupt.py --live` | 否 | 查看已有数据 |
| `analyze_stationary.py` | 否 | 写入/覆盖静止统计 JSON |
| 固定模型应用、普通检测诊断、独立实时录制诊断 | 否 | 创建新的分析结果目录 |
| 数据检查、零偏诊断、姿态与去重力回放 | 否 | 主要在终端输出检查结果 |
| 联合单元测试 | 否 | 输出测试结果，Python 可能生成缓存文件 |
| 新的实时采集 | 是 | 生成新录制；本指南不要求执行 |

本指南不改变原始数据、固件、算法参数或项目阶段。后续如确实产生新录制，再按项目要求登记实际动作与用途。

### English — Quick reference: operational effects

| Operation | Hardware required? | File effects |
| --- | --- | --- |
| Six-axis plots; `diagnose_zupt.py --live` | No | View saved data |
| `analyze_stationary.py` | No | Write/overwrite stationary-statistics JSON |
| Fixed-model application; ordinary detector diagnostics; separate saved-live diagnostics | No | Create new analysis directories |
| Data checks; bias diagnostics; attitude and gravity-removal replay | No | Mainly print terminal results |
| Combined unit tests | No | Print test results; Python may create cache files |
| New live acquisition | Yes | Create new recordings; not required by this guide |

This guide does not change raw data, firmware, algorithm parameters or the project stage. If new recordings are actually created later, register their actual actions and intended uses according to the project rules.

