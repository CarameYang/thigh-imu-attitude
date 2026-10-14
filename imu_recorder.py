# 翻译成英文：收到第一包有效 IMU 数据后，开始计时 60 秒。
# 1. 保存录制期间收到的每个非探测数据包，保留原始字节。
# 2. 保存电脑接收时间，原包里保留 ESP32 时间戳和六轴读数。
# 3. 每秒显示进度，结束后自动关闭文件。
# 4. 继续回复网络 ACK，不把探测消息当作 IMU 数据。

# Start a 60-second timer upon receiving the first valid IMU data packet.
# 1. Save every non-probe packet received during the recording period, preserving the raw bytes.
# 2. Record the computer’s receive time, while retaining the ESP32 timestamp and six-axis sensor readings in the original packet.
# 3. Display progress every second and automatically close the file when recording is complete.
# 4. Continue sending network ACKs, and do not treat probe messages as IMU data.



# 记录 ESP32 通过 UDP 发送的 IMU 实验数据，并生成会话元数据与初步统计。
# Record IMU experiment data sent by an ESP32 over UDP, then generate session
# metadata and preliminary statistics.
#
# 用法示例 / Usage examples:
#   python imu_recorder.py --label stationary --notes "IMU flat on desk"
#   python imu_recorder.py --label motion --notes "Walked along corridor"
#
# 程序会等待第一个有效 IMU 包；收到后才开始 60 秒录制。
# The 60-second recording starts only after the first valid IMU packet arrives.

import argparse
import base64
import json
import math
import socket
import time
from datetime import datetime
from pathlib import Path


# 检查本次实验所需的 IMU 字段，不代替生产端完整协议验证。
# Validate fields needed for this experiment, not the full production schema.
def is_valid_imu(packet):
    # 判断一个已解析的 JSON 值是否为本实验接受的 IMU 包。
    # Return whether a parsed JSON value is an IMU packet accepted by this experiment.
    # 本函数只返回 True 或 False，不修改数据，也不负责打印错误原因。
    # It only returns True or False; it neither changes data nor reports rejection reasons.

    # 顶层必须是 JSON 对象；数组、字符串和数字均不接受。
    # The top level must be a JSON object; arrays, strings, and numbers are rejected.
    if not isinstance(packet, dict):
        return False

    # 限定协议版本、发送节点和数据类型，避免把其他 UDP JSON 误当作 IMU 数据。
    # Restrict protocol version, sender node, and message type so other UDP JSON
    # messages are not mistaken for IMU data.
    if (
        packet.get("version") != 1
        or packet.get("node_id") != "esp32_lab_01"
        or packet.get("type") != "imu"
    ):
        return False

    # 序号和节点时间戳必须是非负整数。使用 type(...) is int 可排除 bool；
    # 在 Python 中 bool 是 int 的子类，因此 isinstance(True, int) 会得到 True。
    # Sequence number and node timestamp must be non-negative integers. Using
    # type(...) is int excludes bool, which isinstance(True, int) would accept.
    for key in ("sequence_number", "timestamp_us"):
        value = packet.get(key)
        if type(value) is not int or value < 0:
            return False

    # 六轴读数位于 data 对象中。
    # The six-axis measurements are stored in the data object.
    data = packet.get("data")
    if not isinstance(data, dict):
        return False

    # 期望三轴加速度单位为 m/s²，三轴角速度单位为 rad/s。
    # Expect three acceleration axes in m/s² and three angular-rate axes in rad/s.
    fields = (
        "ax_mps2", "ay_mps2", "az_mps2",
        "gx_radps", "gy_radps", "gz_radps",
    )

    # 每个字段都必须存在、为普通整数或浮点数，并且不能是 NaN 或正负无穷。
    # Every field must exist, be a regular int or float, and be neither NaN nor infinity.
    return all(
        type(data.get(key)) in (int, float)
        and math.isfinite(data[key])
        for key in fields
    )


def main():
    # 解析命令行参数、接收并保存数据，然后写出会话摘要。
    # Parse command-line arguments, receive and save data, then write a session summary.

    # 用参数区分静止和运动实验，并保存实验说明。
    # Use arguments to distinguish stationary and motion sessions and save notes.
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--label", required=True, choices=("stationary", "motion")
    )
    parser.add_argument("--notes", default="")
    args = parser.parse_args()

    # 目标录制时长固定为 60 秒，从首个有效 IMU 包到达时开始计算。
    # The 60-second target is measured from the arrival of the first valid IMU packet.
    duration_s = 60.0

    # 每次建立独立目录，避免覆盖之前的实验。
    # Create a unique directory for each session to avoid overwriting experiments.
    # 目录名包含到微秒的电脑本地时间和实验标签，例如 20260923_120000_123456_stationary。
    # The directory name contains host local time to microseconds plus the session label.
    folder = (
        Path(__file__).resolve().parent
        / "recordings"
        / (
            datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            + "_"
            + args.label
        )
    )

    # 以下计数器和时间边界用于实时显示及 summary.json 初步统计。
    # These counters and time bounds support live reporting and summary.json statistics.
    fields_count = 0                  # 有效 IMU 包数 / Valid IMU packet count
    invalid_count = 0                 # 无效或其他普通包数 / Invalid or other normal packets
    ack_failures = 0                  # 探测回复发送失败数 / Failed probe ACK transmissions
    start_ns = None                   # 首个有效包到达的单调时间 / Monotonic arrival of first valid packet
    first_arrival_ns = None           # 首个有效包的电脑到达时间 / Host arrival time of first valid packet
    last_arrival_ns = None            # 最后有效包的电脑到达时间 / Host arrival time of last valid packet
    first_node_us = None              # 首个有效包的节点时间戳 / First valid node timestamp
    last_node_us = None               # 最后有效包的节点时间戳 / Last valid node timestamp
    node_time_nonincreasing = 0       # 节点时间未递增次数 / Non-increasing node timestamp count

    # 实时速率按约一秒窗口计算；report_count 保存上次报告时的累计包数。
    # Live rate uses an approximately one-second window; report_count stores the
    # cumulative packet count at the previous report.
    last_report = time.monotonic()
    report_count = 0
    reason = "waiting"

    # 上下文管理器会在正常结束或发生异常时自动关闭 UDP socket。
    # The context manager closes the UDP socket on normal exit or an exception.
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        # 先绑定端口；若被旧接收器占用，此处会报错。
        # Bind first; an existing receiver using this port will cause an error.
        # 0.0.0.0 表示监听电脑所有 IPv4 网络接口上的 UDP 9000 端口。
        # 0.0.0.0 listens on UDP port 9000 on every IPv4 interface of the host.
        sock.bind(("0.0.0.0", 9000))

        # 每 0.1 秒解除一次阻塞，使主循环能检查截止时间并刷新状态。
        # Unblock every 0.1 seconds so the loop can check deadlines and refresh status.
        sock.settimeout(0.1)

        # exist_ok=False 是额外保护：若极少见地目录重名，则报错而不是覆盖。
        # exist_ok=False is an extra safeguard: a rare name collision raises an error.
        folder.mkdir(parents=True, exist_ok=False)

        # session.json 记录实验条件。节点时钟与电脑时钟尚未同步，必须明确说明。
        # session.json records experiment conditions. The node and host clocks are
        # not synchronized, so that limitation is stated explicitly.
        metadata = {
            "created_at": datetime.now().astimezone().isoformat(),
            "label": args.label,
            "notes": args.notes,
            "duration_target_s": duration_s,
            "node_id": "esp32_lab_01",
            "hardware": "DOIT ESP32 DEVKIT V1 + SparkFun ICM-20948",
            "timestamp_note": (
                "Host receive timestamps are recorded after recvfrom; "
                "node and host clocks are not synchronized."
            ),
        }

        # UTF-8 和 ensure_ascii=False 允许 notes 中的中文原样写入。
        # UTF-8 and ensure_ascii=False preserve Chinese text in notes.
        (folder / "session.json").write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )

        print(f"Output folder: {folder}", flush=True)
        print("Waiting for the first valid IMU packet...", flush=True)

        # 等待阶段最多 30 秒；这段时间不计入正式的 60 秒录制。
        # Wait up to 30 seconds; this waiting period is not part of the 60-second session.
        waiting_started = time.monotonic()

        # JSONL 每行一个数据包；原始字节用 Base64 无损保存。
        # JSONL stores one packet per line; Base64 preserves the exact raw bytes.
        # 模式 "x" 要求文件不存在，进一步避免意外覆盖。
        # Mode "x" requires a new file, providing another overwrite safeguard.
        with (folder / "packets.jsonl").open(
            "x", encoding="utf-8"
        ) as output:
            try:
                while True:
                    # monotonic_ns 不受系统时间手动调整或网络校时跳变影响，适合算时长。
                    # monotonic_ns is unaffected by wall-clock corrections, so it is
                    # suitable for durations.
                    now_ns = time.monotonic_ns()

                    # 收到首个有效包后检查 60 秒截止；否则检查 30 秒等待上限。
                    # After the first valid packet, enforce the 60-second deadline;
                    # otherwise enforce the 30-second initial wait limit.
                    if start_ns is not None:
                        if (now_ns - start_ns) / 1e9 >= duration_s:
                            reason = "completed"
                            break
                    elif time.monotonic() - waiting_started >= 30.0:
                        reason = "no_valid_imu_within_30s"
                        print("No valid IMU data within 30 seconds.")
                        break

                    try:
                        # 65535 是一次 UDP 数据报可接收缓冲区的上限值；sender 为 (IP, port)。
                        # 65535 is the receive buffer limit used for one UDP datagram;
                        # sender contains its source (IP, port).
                        raw, sender = sock.recvfrom(65535)

                        # 收包后立即记录电脑时间，不以写文件时间代替。
                        # Capture host times immediately after receiving the packet.
                        # monotonic 时间用于间隔/速率；Unix 时间用于对应现实日期时间。
                        # Monotonic time supports intervals/rates; Unix time maps to
                        # real-world date and time.
                        arrival_ns = time.monotonic_ns()
                        arrival_unix_ns = time.time_ns()

                    except socket.timeout:
                        # 超时并非错误，只表示过去 0.1 秒没有收到包。
                        # A timeout is not an error; it only means no packet arrived in 0.1 s.
                        raw = None

                    if raw is not None:
                        # 超过录制截止时间的包不纳入本次记录。
                        # Exclude packets received after the recording deadline.
                        if (
                            start_ns is not None
                            and arrival_ns - start_ns >= int(duration_s * 1e9)
                        ):
                            reason = "completed"
                            break

                        # 探测包单独回复，不计入 IMU 样本。
                        # Reply to probes separately without counting them as IMU samples.
                        if raw.startswith(b"DIP_PING "):
                            token = raw[len(b"DIP_PING "):]

                            # 只回复恰好 16 个小写十六进制字符的 token，拒绝畸形探测包。
                            # Reply only to a token of exactly 16 lowercase hexadecimal
                            # characters; malformed probes are ignored.
                            if (
                                len(token) == 16
                                and all(
                                    value in b"0123456789abcdef"
                                    for value in token
                                )
                            ):
                                try:
                                    sock.sendto(b"DIP_ACK " + token, sender)
                                except OSError:
                                    # 回复失败不终止 IMU 录制，只累计到摘要。
                                    # An ACK failure does not stop recording; it is summarized.
                                    ack_failures += 1
                        else:
                            try:
                                # UDP 原始字节先按 UTF-8 解码，再解析为 JSON 并验证字段。
                                # Decode UDP bytes as UTF-8, parse JSON, then validate fields.
                                packet = json.loads(raw.decode("utf-8"))
                                valid = is_valid_imu(packet)
                            except (ValueError, UnicodeError, OverflowError):
                                # 包括非法 UTF-8、非法 JSON 或数值转换溢出等情况。
                                # Covers invalid UTF-8, malformed JSON, and numeric overflow.
                                packet = None
                                valid = False

                            if valid and start_ns is None:
                                # 首个有效 IMU 包同时定义正式录制的时间零点。
                                # The first valid IMU packet defines time zero for recording.
                                start_ns = arrival_ns
                                print(
                                    "Recording started: keep the planned condition "
                                    "for 60 seconds.",
                                    flush=True,
                                )

                            # 无效包也保留原文，便于检查，不能静默丢弃。
                            # Preserve invalid packets as well for later inspection.
                            # packet 只为有效包保存解析结果；raw_base64 始终保存精确原始字节。
                            # Parsed packet data is stored only when valid; raw_base64 always
                            # preserves the exact received bytes.
                            record = {
                                "arrival_unix_ns": arrival_unix_ns,
                                "arrival_monotonic_ns": arrival_ns,
                                "sender_ip": sender[0],
                                "sender_port": sender[1],
                                "valid_imu": valid,
                                "raw_base64": base64.b64encode(raw).decode("ascii"),
                                "packet": packet if valid else None,
                            }

                            # 一行写一个完整 JSON 对象，便于流式读取和故障后保留已写数据。
                            # Write one complete JSON object per line for streaming reads and
                            # retention of already-written records after an interruption.
                            output.write(
                                json.dumps(record, ensure_ascii=False) + "\n"
                            )

                            if valid:
                                fields_count += 1
                                node_us = packet["timestamp_us"]

                                # 首个有效样本只设置一次起点，后续样本持续更新终点。
                                # Set the start boundary once and update the end boundary
                                # for every subsequent valid sample.
                                if first_arrival_ns is None:
                                    first_arrival_ns = arrival_ns
                                    first_node_us = node_us

                                # 相等或倒退都计为“未递增”，提示节点计时可能重复或回退。
                                # Equal or decreasing values both count as non-increasing,
                                # indicating a duplicate or backward node time.
                                if (
                                    last_node_us is not None
                                    and node_us <= last_node_us
                                ):
                                    node_time_nonincreasing += 1

                                last_arrival_ns = arrival_ns
                                last_node_us = node_us
                            else:
                                invalid_count += 1

                    # 每秒显示一次，但文件仍逐包保存。
                    # Display once per second while saving every packet.
                    now = time.monotonic()
                    if now - last_report >= 1.0:
                        # 此处是最近一个报告窗口的接收速率，不是整个会话平均值。
                        # This is the receive rate for the latest reporting window,
                        # not the average across the whole session.
                        rate = (
                            fields_count - report_count
                        ) / (now - last_report)

                        # 未收到首个有效包时 elapsed 保持 0.0 秒。
                        # elapsed remains 0.0 until the first valid packet arrives.
                        elapsed = (
                            (time.monotonic_ns() - start_ns) / 1e9
                            if start_ns is not None else 0.0
                        )

                        print(
                            f"{elapsed:5.1f}/60 s | rx={rate:.1f} Hz | "
                            f"saved_imu={fields_count} | "
                            f"invalid_or_other={invalid_count}",
                            flush=True,
                        )

                        # 每秒刷新用户态缓冲区，减少异常中止时尚未交给操作系统的数据量。
                        # Flush the user-space buffer each second to reduce unwritten data
                        # if the process stops unexpectedly.
                        output.flush()
                        last_report = now
                        report_count = fields_count

            except KeyboardInterrupt:
                # Ctrl+C 属于用户主动提前结束，仍继续生成 summary.json。
                # Ctrl+C is an intentional early stop; summary.json is still generated.
                reason = "stopped_by_user"

        # 保存初步统计；详细缺号与间隔检查在录制后进行。
        # Save preliminary statistics; inspect sequence gaps and intervals afterward.
        arrival_rate = None
        if (
            fields_count > 1
            and last_arrival_ns > first_arrival_ns
        ):
            # N 个样本之间有 N-1 个到达间隔，因此分子使用 fields_count - 1。
            # N samples contain N-1 arrival intervals, hence fields_count - 1.
            arrival_rate = (
                (fields_count - 1)
                / ((last_arrival_ns - first_arrival_ns) / 1e9)
            )

        # summary.json 是快速概览；它不替代后续对序号缺口、重复、乱序和
        # 每包时间间隔的完整离线分析。
        # summary.json is a quick overview; it does not replace later offline analysis
        # of sequence gaps, duplicates, reordering, and per-packet intervals.
        summary = {
            "ended_at": datetime.now().astimezone().isoformat(),
            "reason": reason,
            "valid_imu_packets": fields_count,
            "invalid_or_other_packets": invalid_count,
            "ack_send_failures": ack_failures,
            "arrival_rate_hz": arrival_rate,
            "first_node_timestamp_us": first_node_us,
            "last_node_timestamp_us": last_node_us,
            "node_timestamp_nonincreasing_count": node_time_nonincreasing,
        }

        (folder / "summary.json").write_text(
            json.dumps(summary, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )

        print("\nRecording ended:", reason)
        print("Valid IMU packets:", fields_count)
        if arrival_rate is not None:
            print(f"Average arrival rate: {arrival_rate:.2f} Hz")
        print("Saved to:", folder)


# 直接运行本文件时才调用 main；被其他 Python 文件导入时不会自动开始监听。
# Call main only when this file is executed directly; importing it will not start listening.
if __name__ == "__main__":
    main()
