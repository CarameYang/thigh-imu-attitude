# 多记录训练与验证 / Multi-recording fitting and validation

运行 / Run:

```powershell
python "<repo-root>\train_zupt.py"
```

仅使用标准库，不需安装依赖。`train_zupt.py` 与 `zupt_offline.py` 放在同一目录。
Standard library only. Keep `train_zupt.py` beside `zupt_offline.py`.

## 数据划分 / Split

`zupt_split.json` 保存用户确认的静止标签。训练：两份 stationary 全段，最新 motion 的 [55,60) 秒。验证：四份 motion 各自的 [5,10) 秒。全部时间相对各自首个节点样本，末端不包含。最新 motion 为大腿安装；不能将旧元数据中的鞋部描述当作当前事实。
`zupt_split.json` stores user-confirmed stationary labels. Train on both entire stationary sessions and [55,60) s of the latest motion session; validate on [5,10) s of each of the four motion sessions. Times are relative to each recording's first node sample. Latest motion mounting is the thigh; historical foot-mount metadata is not authoritative for it.

程序分别统计每个训练区间，再对各区间的陀螺仪均值等权平均得到一个待验证的共用偏置。加速度不减各轴均值。阈值为各训练区间特征 99% 分位数的最大值乘 1.5，并使用配置中的下限；这些是工程初始参数，不是保证有效的通用值。不会因为验证结果不好而自动调宽阈值。
Each training interval is summarized separately; interval gyro means are equally weighted into a provisional shared bias. Acceleration axis means are not subtracted. Thresholds use the maximum per-interval 99th percentile times 1.5, with configured floors. These are initial engineering choices, not universal guarantees. Poor validation does not automatically relax thresholds.

完整 0.2 秒窗口必须全部落在标签内。缺号或超过 0.04 秒间隔会重置窗口。被排除的窗口末端数量单独记录；不是全部都代表丢包，也包含区间开始的预热窗口。不同记录不拼接。
Entire 0.2 s windows must lie within labeled intervals. Missing IDs or gaps over 0.04 s reset the window. Excluded endpoints include warm-up windows and do not all imply packet loss. Sessions are never concatenated.

## 输出 / Outputs

每次创建独立 `analysis/zupt_training_<时间>/`，保存：
Each run creates a separate output folder containing:

- `model.json`：训练段统计、固定偏置、阈值、方法、源数据哈希。Training statistics, fixed bias, thresholds, method and source hashes.
- `split.json`：本次标签划分副本。Snapshot of the labeled split.
- `validation.json`：每份验证记录的原始/补偿后均值、RMS、可用窗口、静止检出比例和各判据失败数量。Per-session residuals, RMS, eligible windows, stationary detection fractions and feature failures.
- `summary.txt`：双语结果摘要。Bilingual summary.
- `<记录名>_detections.jsonl`：每份记录全段的时间、序号、特征及候选/未知状态。Full-session timestamps, sequences, features and candidate/unknown decisions.

复用参数时使用以下形式（将模型路径和录制目录换为实际路径）：
Reuse fitted parameters with actual model and recording paths:

```powershell
python train_zupt.py --model "analysis/zupt_training_<时间>/model.json" --session "recordings/<录制目录>"
```

这一步只应用保存模型，不重新估计偏置或阈值。旧的 `zupt_offline.py` 直接运行仍使用原先单记录方案，不会自动加载新模型。
This applies the saved model without refitting. Running the old `zupt_offline.py` directly still uses its original single-session procedure; it does not automatically load the new model.

## 结果边界 / Interpretation

验证均为用户确认的静止片段，检出比例不是整体准确率；运动误触发率尚不能评价。重叠滑窗高度相关，不能视为独立实验次数。最新记录两段虽不重叠，但属于同一次实验。共用偏置如果让某份验证残差变差，应保留结果并调查，不把验证段重新纳入训练后继续声称独立验证。所有输出仅用于静止检测准备，不包含 INS、速度更新或位置轨迹。
Validation covers only user-confirmed stationary intervals, so detection fractions are not overall accuracy and do not measure motion false positives. Overlapping windows are correlated, not independent trials. Disjoint intervals of the latest recording still belong to one session. If shared bias worsens a validation residual, retain and investigate that result; do not refit on it and still claim independent validation. Outputs are detector preparation only, not INS, velocity updates or trajectories.
