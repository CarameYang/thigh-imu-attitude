# Thigh-Mounted IMU Attitude Estimation and Gravity Removal

A complete, replayable bench pipeline from a consumer six-axis IMU (ICM-20948)
to world-frame linear acceleration — with explicit scope limits.

**Status as of 2026-10-14 14:55 (Asia/Shanghai, UTC+08:00).**
This document describes the implementation that actually exists at that time,
not a planned or expected state.

> **In one line**
> Done: real six-axis acquisition → raw recording → integrity and timing checks
> → stationary candidate detection → gyro bias → attitude estimation → formal
> gravity removal → replay validation → one-step velocity propagation.
> Not done: actual ZUPT updates, position propagation, ground-truth-validated
> trajectory accuracy, UWB fusion.

---

## Table of contents

- [1. What this is](#1-what-this-is)
- [2. Hardware and environment](#2-hardware-and-environment)
- [3. Data flow](#3-data-flow)
- [4. Quick start](#4-quick-start)
- [5. Repository layout](#5-repository-layout)
- [6. Documentation map](#6-documentation-map)
- [7. Key conventions](#7-key-conventions)
- [8. Results and evidence](#8-results-and-evidence)
- [9. Tests](#9-tests)
- [10. Data and privacy](#10-data-and-privacy)
- [11. Limitations and common misreadings](#11-limitations-and-common-misreadings)
- [12. Roadmap](#12-roadmap)
- [13. Provenance and scope](#13-provenance-and-scope)
- [14. License](#14-license)
- [15. Pre-publish checklist](#15-pre-publish-checklist)

---

## 1. What this is

This repository holds a standalone **thigh-mounted IMU sub-module** for indoor
trajectory estimation of emergency responders, where inertial sensing must
bridge short gaps without external ranging.

**Important**: this is **not** a working positioning system. What it does is
acquire real IMU data reliably, compute world-frame linear acceleration from it,
and quantify the remaining error honestly.

### Current status

| Capability | Status | Evidence |
|---|---|---|
| Real six-axis acquisition (ESP32 → UDP → host) | ✅ Verified | Real `packets.jsonl` recordings, continuous sequence numbers |
| Raw data fidelity (raw bytes + parsed values) | ✅ Verified | base64 raw packets match the parsed result |
| Node time separated from host receive time | ✅ Verified | Node intervals 19.556–20.449 ms (≈ 50 Hz) |
| Gyro bias calibration | ✅ Verified | Calibrated per recording, never reused across sessions |
| Attitude estimation (Mahony quaternion complementary filter) | ✅ Implemented and tested | 22 attitude tests |
| Gravity removal (world-frame linear acceleration) | ✅ Implemented and tested | 8 tests + 16-window real-data replay |
| Stationary candidate detection (`zupt_allowed`) | ✅ Implemented and tested | Fixed model + 11 detection/training tests |
| Velocity propagation (world-frame trapezoidal integration) | 🟡 Just implemented, accuracy unverified | 8 tests; stationary drift diagnostics only |
| Actual ZUPT velocity update | ❌ Not implemented | A candidate is not an update |
| Position propagation / trajectory | ❌ Not implemented | No trajectory results at all |
| Dynamic accuracy against ground truth | ❌ Unverified | No independent reference trajectory |
| Full IMU calibration (temperature, scale, mounting angle) | ❌ Incomplete | Nominal mounting direction only |
| UWB / fusion / live UI | ❌ Out of scope here | Owned by other parts of the wider system |

### Design principles

1. **Raw data is never overwritten.** No processing rewrites a recording;
   results are stored separately.
2. **Measurement time is not arrival time.** Attitude integration uses node time
   differences only; host arrival jitter is never treated as motion.
3. **A candidate is not an update.** `zupt_allowed` means "permission to
   consider a zero-velocity observation", not "velocity has been zeroed".
4. **Nothing is deleted or tuned to make a curve look good.** Spikes, persistent
   offsets and failing windows are all retained.
5. **Passing tests is not passing accuracy.** Unit tests prove computation
   behaviour, not real-world motion accuracy.

---

## 2. Hardware and environment

| Item | Value |
|---|---|
| MCU | DOIT ESP32 DEVKIT V1 (ESP-WROOM-32, CP2102 USB-serial) |
| IMU | SparkFun Qwiic 9DoF IMU **ICM-20948** (I²C, six axes used) |
| Firmware | `imu_udp_50hz_test` (Arduino, not in this repository; ~50 Hz UDP) |
| Port | UDP 9000 (host receiver); node timestamps are integer microseconds |
| Node ID | `esp32_lab_01` (validated by `is_valid_imu`) |
| Mounting | **Thigh-mounted**; hand-held recordings never define mounting |
| Verified on | Python 3.14.6 (`pyproject.toml` declares `requires-python >= 3.14`) |
| Dependencies | **None** — Python standard library only |

> **Planned final hardware** (unchanged by these bench experiments):
> ICM-42688-P foot/thigh IMU and 4 × Qorvo DWM3000EVB with 4 × ESP32
> (3 anchors + 1 tag).

Firmware unit conversion (read-only verified): acceleration `mg × 0.00980665`
→ m/s²; angular rate `deg/s × 0.01745329252` → rad/s. The firmware preserves the
raw sensor axis mapping and never substitutes world coordinates.

---

## 3. Data flow

```text
+--------------------------- Acquisition ----------------------------+
|  ICM-20948 --I2C--> ESP32 --UDP/JSON, ~50 Hz--> host UDP 9000      |
|                      (imu_udp_50hz_test firmware, not in this repo)|
+----------------------------------+---------------------------------+
                                   |
                    +---------------------------+
                    |                           |
             imu_recorder.py
                                      live_zupt_receiver.py
      (timed recording, raw bytes)
                              (live stationary candidates + state)
                    |                           |
                    +---------------------------+
                                   v
         recordings/  .  live_recordings/    <- not committed
      packets.jsonl . states.jsonl . session.json . summary.json
                                   |
+------------------- Offline replay and analysis --------------------+
|  check_pregravity.py      integrity, sequence, node intervals      |
|  attitude.py              gyro bias + Mahony quaternion q_bw       |
|  gravity_removal.py       a_world = R_bw.R_BS.f + (0,0,-9.80665)   |
|  stationary_stream.py     stationary candidate state machine       |
|  velocity_propagation.py  one-step trapezoidal velocity (no ZUPT)  |
|  validate_gravity_removal.py / validate_velocity_drift.py          |
+----------------------------------+---------------------------------+
                                   v
       analysis/<topic>_<timestamp>/   <- text reports committed
                         jsonl output excluded
```

---

## 4. Quick start

### 4.1 Environment

```powershell
# Only Python 3.14+ is required; no third-party packages.
python --version
```

### 4.2 Run the tests

38 tests run without any recording (22 attitude + 8 gravity + 8 velocity):

```powershell
python -m unittest test_attitude test_gravity_removal test_velocity_propagation
```

The full 49 (the other 11 need local `recordings/` and the fixed model, see §9):

```powershell
python -m unittest test_gravity_removal test_attitude test_stationary_stream `
                   test_zupt_offline test_train_zupt test_velocity_propagation
```

### 4.3 Read the existing reports

The core results of this repository are **written reports**. Start with these two:

| Report | Contents |
|---|---|
| [IMU stage assessment and detailed record](analysis/imu_stage_review_20261014_0000/IMU阶段评估与详细记录.md) | Method, the full 16-window result table, the three residual decompositions, limits |
| [Gravity-removal validation results and reproduction guide](2026-10-14_1436_Gravity_Removal_Validation_Results_and_Reproduction_Guide.md) | Stage-by-stage reproduction commands, expected output, presentation order |

> Both reports are written in Chinese. An English translation has not been
> produced yet.

### 4.4 Reproduce with your own data

```powershell
# 1) Record first (hardware required; the timer starts on the first valid packet)
python imu_recorder.py --label stationary --notes "IMU flat on desk"

# 2) Integrity, sequence and node-interval checks
python -X utf8 check_pregravity.py

# 3) 16-window gravity-removal statistics (uses four fixed recordings)
python -X utf8 validate_gravity_removal.py

# 4) Inspect the six-axis traces
python -X utf8 visualize_imu.py --recordings live_recordings
```

> `-X utf8` keeps the programs' Chinese terminal output correct on Windows.

---

## 5. Repository layout

```text
.
├── README.md                     <- this file
├── LICENSE                       <- MIT
├── .gitignore
├── pyproject.toml                <- requires-python >= 3.14, no dependencies
│
├── docs/                         <- documentation set
│   ├── architecture.md           module responsibilities and boundaries
│   ├── coordinates.md            frames, mounting, quaternion conventions
│   ├── data-formats.md           UDP packet and recording file fields
│   ├── algorithms.md             attitude / gravity removal / detection / velocity
│   ├── experiments.md            recording inventory, method, evidence rules
│   ├── limitations.md            limits, unfinished work, common misreadings
│   └── development.md            environment, tests, commits, data policy
│
├── Acquisition and receivers
│   ├── imu_recorder.py           timed recording, raw bytes and arrival times
│   ├── live_zupt_receiver.py     live reception + stationary candidates + state log
│   ├── udp_rate_receiver.py      UDP arrival-rate measurement
│   ├── udp_test_receiver.py      minimal UDP link test
│   └── update_imu_reconnect.py   recording transfer/collection utility
│
├── Algorithm modules
│   ├── attitude.py               quaternion Mahony filter, R_BS, Euler angles
│   ├── gravity_removal.py        pure gravity removal (no network/filter/integration)
│   ├── stationary_stream.py      per-packet stationary candidate state machine
│   ├── velocity_propagation.py   one-step world-frame trapezoidal integration
│   ├── zupt_offline.py           offline bias and candidate detection (single session)
│   └── train_zupt.py             multi-recording fitting/validation, emits fixed model
│
├── Checks and diagnostics
│   ├── check_pregravity.py       integrity, timing, attitude internal consistency
│   ├── check_attitude_replay.py  attitude + gravity-removal replay, 4 recordings
│   ├── check_recording_direction.py  detect still/turn intervals, decide direction
│   ├── verify_direction.py       vertical-axis turn test for the sensor +X 180° ambiguity
│   ├── diagnose_gyro_bias.py     calibration-window sensitivity
│   ├── diagnose_zupt.py          detector diagnostics for a fixed recording
│   ├── diagnose_live_zupt.py     standalone diagnostic report for a live segment
│   ├── diagnose_horizontal_drift.py  horizontal residual/drift diagnostics
│   ├── compare_initial_residual.py   fixed world offset fitted on the initial still data
│   ├── analyze_stationary.py     early stationary-recording statistics
│   └── visualize_imu.py          interactive six-axis viewer
│
├── Validation
│   ├── validate_gravity_removal.py   16-window mean/RMS/p95/peak
│   └── validate_velocity_drift.py    velocity drift on still plateaus
│
├── Tests (6 modules, 49 tests)
│   ├── test_attitude.py              22
│   ├── test_gravity_removal.py        8
│   ├── test_velocity_propagation.py   8
│   ├── test_stationary_stream.py      3   <- requires recordings
│   ├── test_zupt_offline.py           4   <- requires recordings
│   └── test_train_zupt.py             4   <- requires recordings
│
├── zupt_split.json               <- user-confirmed still labels (train/validation)
│
├── analysis/                     <- 16 analysis output directories, reports committed
│   ├── imu_stage_review_20261014_0000/   stage assessment + window metrics + file hashes
│   ├── attitude_validation_20261012/     evidence for the mounting-direction decision
│   ├── mounting_check_20261012_003433/   worn-mounting check
│   ├── gyro_check_20261012_004616/       gyro sign and scale cross-check
│   ├── zupt_training_20261007_164450_519158/  the fixed detector model.json
│   └── ... (remaining directories are diagnostics and replays)
│
└── Project records
    ├── RECORDINGS_INDEX.md       what was actually done in each recording
    └── ZUPT_TRAINING_README.md   detector training notes
```

**Not committed**: `recordings/`, `live_recordings/`, `analysis/**/*.jsonl`,
`downloads/`, `repository-review/`, `.venv/`, `.idea/`, `.document_work/`,
`*.docx`, `personal.ipynb`, and the internal development logs (`AGENTS.md`,
`PROGRESS.md`, `NEXT_STEPS.md`). See [.gitignore](.gitignore).

---

## 6. Documentation map

| To learn about | Read |
|---|---|
| What each module does and where its boundaries are | [docs/architecture.md](docs/architecture.md) |
| Coordinate frames, mounting, quaternion conventions | [docs/coordinates.md](docs/coordinates.md) |
| UDP packet and recording file fields | [docs/data-formats.md](docs/data-formats.md) |
| Attitude, gravity removal, detection and velocity algorithms | [docs/algorithms.md](docs/algorithms.md) |
| Which experiments exist and what they can prove | [docs/experiments.md](docs/experiments.md) |
| What is still missing, and what must not be claimed | [docs/limitations.md](docs/limitations.md) |
| Environment, tests, commits and data policy | [docs/development.md](docs/development.md) |
| Real results and reproduction steps | [stage assessment](analysis/imu_stage_review_20261014_0000/IMU阶段评估与详细记录.md), [reproduction guide](2026-10-14_1436_Gravity_Removal_Validation_Results_and_Reproduction_Guide.md) |
| What was actually done in each recording | [RECORDINGS_INDEX.md](RECORDINGS_INDEX.md) |
| Detector training details | [ZUPT_TRAINING_README.md](ZUPT_TRAINING_README.md) |

---

## 7. Key conventions

| Convention | Value |
|---|---|
| Sensor mounting (nominal) | **+X up, +Y left, +Z back** (silkscreen face away from the leg) |
| Body frame | +x forward, +y left, +z up (right-handed) |
| Mounting transform | `body = (−sensor Z, sensor Y, sensor X)`, i.e. `attitude.R_BS`, determinant +1 |
| World frame | +z up; the horizontal heading is chosen at initialization and is **not true north** |
| Quaternion | `(w, x, y, z)`, meaning **body to world**, `q_bw` |
| Units | m, m/s, m/s², rad, rad/s; gravity constant 9.80665 m/s² |
| Timestamps | Integer node microseconds, `timestamp_us`; **node and host clocks are not synchronized** |
| Gravity removal | `a_world = R_bw · R_BS · f_sensor + (0, 0, −9.80665)` |

> ⚠️ Historical descriptions mapping **sensor +X to body −z are superseded** —
> they do not match the current definition or the code. Full derivation and
> evidence: [docs/coordinates.md](docs/coordinates.md).

---

## 8. Results and evidence

Based on four real recordings from 2026-10-12 (**13,478** valid packets in total,
node intervals 19.556–20.449 ms, ≈ 50 Hz), replaying gravity removal over
**fixed windows**:

| Metric | Result | Caveat |
|---|---|---|
| Windows with all three axis RMS < 0.2 m/s² (of 16) | **15 / 16** | A diagnostic window count, **not 93.75 % accuracy**; 0.2 is not an agreed acceptance threshold |
| Hand-held `025851`, 8.5–10 s: X mean / RMS | −0.1814 / 0.2123 m/s² | Both half-windows exceed the line, so a single spike does not explain it |
| Thigh `003433`, 31–39.5 s: Z absolute peak | 1.6862 m/s² | Overall RMS stays small, so small RMS coexists with spikes |
| Tabletop `004616`, 45–59 s: X absolute peak | 1.0920 m/s² | The adjacent change comes mainly from input measurements, not an attitude jump |
| Estimated heading change of the slow body turn | ≈ 90.02° | Consistent with the ≈ 90° action, but the action has no precise reference — **this is not a 0.02° error** |
| Turn estimate across calibration windows | 90.02 / 87.48 / 84.44 / 79.59° | The result is window-sensitive; picking the value closest to 90° does not establish accuracy |
| Residual heading change during still intervals | ≈ 7.7° / 12.3° / 13.5° | A six-axis IMU has no absolute heading, so rotation accumulates even at rest |

All 16 windows and all samples are retained; nothing was filtered by detector
output and no samples were deleted.

**These numbers show internal consistency, not dynamic accuracy.** Attitude
estimation reuses the same accelerometer, so still-interval residuals cannot
independently prove motion acceleration or trajectory accuracy.

---

## 9. Tests

Last combined run (2026-10-14, local): **49 passed**.

| Test module | Tests | Needs recordings | Coverage |
|---|---:|---|---|
| `test_attitude.py` | 22 | No | Mounting determinant, integration sign, quaternions, physical regression, stage readiness |
| `test_gravity_removal.py` | 8 | No | Rest, tilt/inversion, real linear-acceleration preservation, turning, free fall, invalid time and values |
| `test_velocity_propagation.py` | 8 | No | Trapezoidal integration math, reversed/duplicate time, gap timeout, explicit initialization and external correction |
| `test_stationary_stream.py` | 3 | **Yes** | State-machine safety gates and replay consistency |
| `test_zupt_offline.py` | 4 | **Yes** | Key offline detector behaviour |
| `test_train_zupt.py` | 4 | **Yes** | Training isolation and interval boundaries |

> **11 tests fail in a fresh clone**, because raw recordings are intentionally
> not committed. To reproduce them, supply your own `recordings/` plus
> `analysis/zupt_training_20261007_164450_519158/model.json`.

---

## 10. Data and privacy

- **Raw experiment recordings are not published.** Recordings, per-packet
  output and replay artifacts are excluded by [.gitignore](.gitignore) and
  remain on the author's machine — not deleted, just not committed.
- **No credentials.** A full scan found no API keys, tokens, passwords or
  private keys.
- **Nothing that points to an individual is published.** Host user names and
  absolute paths, LAN addresses, personal names, institution and related
  repository addresses were removed and verified by a full-text scan of every
  committed file.
- **One deliberate exception: the firmware protocol constant.** `DIP_PING` and
  `DIP_ACK` in the code are the handshake bytes agreed with the ESP32 firmware.
  Renaming them would immediately break communication with the hardware, so they
  are kept verbatim; they are opaque protocol literals and point to no person or
  organization.

---

## 11. Limitations and common misreadings

**Do not conclude the following from this repository:**

| ❌ Incorrect claim | ✅ Actual situation |
|---|---|
| "Indoor positioning has been implemented" | The pipeline stops at world-frame linear acceleration and one-step velocity integration; there is no position and no trajectory |
| "ZUPT is done" | Only stationary **candidates** exist; the actual zero-velocity update is not implemented |
| "15/16 means 93.75 % accuracy" | That is a diagnostic window count, unrelated to positioning accuracy |
| "The turn error is only 0.02°" | The action itself has no precise reference, and the result is sensitive to the calibration window |
| "A still thigh means zero velocity" | Low thigh dynamics ≠ zero velocity at foot strike ≠ zero instantaneous IMU velocity |
| "A six-axis IMU gives heading" | There is no absolute heading reference, so rotation accumulates even at rest |
| "All tests pass, so accuracy is fine" | Unit tests cover computation and input guards, not real-motion accuracy |
| "Hand-held data can determine the mounting direction" | A hand-held orientation is decided by the hand and is not mounting evidence |

The complete boundary list and the physical-cause analysis are in
[docs/limitations.md](docs/limitations.md).

---

## 12. Roadmap

In the project's existing order (unfinished items; this is not a schedule):

1. **Velocity propagation** (in progress): validate the integration math with
   synthetic inputs, then quantify drift on still recordings; define initial
   velocity, validity, gap handling and the external corrected-state interface.
2. **Position propagation**: short walk-stop replays; then arrange route trials
   with real distances where ground truth is otherwise missing.
3. **Actual ZUPT updates**: agree timing, frames, corrected-state interfaces and
   ownership with the fusion owner.
4. **Independent dynamic accuracy validation**: requires an external reference
   (measured routes, action-time ground truth).
5. **Full IMU calibration**: temperature drift, scale factors, precise mounting
   angles (currently only a nominal direction).
6. **Handover and integration**: deliver six-axis data, calibration/mounting
   documentation, a propagation module that accepts external corrections, and
   the stationary candidate material.

---

## 13. Provenance and scope

This repository is a standalone record of a **wearable IMU sub-module**: the
pipeline from thigh-mounted six-axis data to world-frame linear acceleration.
The wider system (UWB ranging, state estimation and fusion, backend and live UI)
is out of scope here.

When integrating with an external system, that system's contracts take
precedence. The UDP field names here follow the common
`version` / `node_id` / `sequence_number` / `timestamp_us` / `type` / `data`
layout, but the local validator intentionally checks only the fields this
experiment needs.

---

## 14. License

[MIT](LICENSE) © 2026 CarameYang.

---

## 15. Pre-publish checklist

**Identity information has been handled** (completed 2026-10-14, verified by
scanning every file):

- [x] Host user names and absolute paths (`C:\Users\<user>\…`) → replaced with placeholders.
- [x] LAN addresses (`192.168.x.x`, `172.x.x.x`) → replaced with placeholders.
- [x] Personal names, institution and related repository addresses → removed.
- [x] Internal development logs (`AGENTS.md`, `PROGRESS.md`, `NEXT_STEPS.md`) →
      excluded via [.gitignore](.gitignore), kept locally only.
- [x] Local documents (`*.docx`) → not published.
- [x] Retained `DIP_PING` / `DIP_ACK` are firmware handshake constants; renaming them would break the hardware link.

Still to confirm manually before pushing more:

- [ ] **Publication permission**: whether this content and these measured results are appropriate to publish.
- [ ] **No oversized files**: check `git status` for anything > 50 MB
      (GitHub's hard per-file limit is 100 MB).
- [ ] **No credentials**: read through `git diff --cached`.
- [ ] **Recordings not committed**: `recordings/`, `live_recordings/`, `*.jsonl` are ignored.
- [ ] **Internal logs not committed**: `AGENTS.md`, `PROGRESS.md`, `NEXT_STEPS.md`
      are absent from the staged list.
- [ ] **Update the status date**: edit "Status as of" at the top and in the §1 table.

---

<div align="center">

**Accurate statement of this stage**

We have completed real IMU acquisition and initial attitude and gravity-removal
validation, while retaining and analyzing residuals. These results support
continuing velocity-propagation development, but we do **not** yet have a
motion trajectory validated against independent ground truth.

</div>
