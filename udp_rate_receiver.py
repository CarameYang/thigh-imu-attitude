import json
import socket
import time

# 本测试只统计指定节点的 IMU 数据。
# Count IMU packets from the selected node only.
NODE_ID = "esp32_lab_01"

# 用集合区分唯一序号、重复包和乱序包。
# Use a set to distinguish unique sequences, duplicates and reordering.
seen = set()
first_seq = None
highest_seq = None
duplicates = 0
out_of_order = 0
invalid = 0
window_count = 0
last_packet_time = None
latest = None

# 每秒显示一次，不逐包打印。
# Report once per second instead of printing every packet.
report_time = time.monotonic()

with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
    sock.bind(("0.0.0.0", 9000))
    sock.settimeout(0.1)

    print("Listening on UDP 9000. Ctrl+C to stop.")
    print("Restart this receiver whenever the ESP32 restarts.")

    try:
        while True:
            try:
                raw, sender = sock.recvfrom(4096)

                # 网络探测包单独处理，不计入 IMU 频率或格式错误。
                # Handle probes separately from IMU rate and validation statistics.
                if raw.startswith(b"DIP_PING "):
                    token = raw[len(b"DIP_PING "):]

                    # 只回复固定长度的十六进制编号。
                    # Reply only to fixed-length hexadecimal tokens.
                    if (
                        len(token) == 16
                        and all(
                            value in b"0123456789abcdef"
                            for value in token
                        )
                    ):
                        try:
                            # 回复到探测包的实际来源地址和端口。
                            # Reply to the probe's actual source address and port.
                            sock.sendto(b"DIP_ACK " + token, sender)
                        except OSError as error:
                            print(
                                f"ACK send failed: {error}",
                                flush=True,
                            )

                    continue

                # 检查本测试需要的字段；这不是生产协议验证器。
                # Check fields needed for this test, not the full production schema.
                packet = json.loads(raw.decode("utf-8"))

                if not isinstance(packet, dict):
                    raise ValueError("Expected an object")

                if (
                    packet.get("node_id") != NODE_ID
                    or packet.get("type") != "imu"
                ):
                    continue

                seq = packet["sequence_number"]
                data = packet["data"]

                if type(seq) is not int or seq < 0:
                    raise ValueError("Invalid sequence")

                fields = (
                    "ax_mps2", "ay_mps2", "az_mps2",
                    "gx_radps", "gy_radps", "gz_radps",
                )

                if not isinstance(data, dict) or not all(
                    type(data.get(key)) in (int, float)
                    for key in fields
                ):
                    raise ValueError("Invalid IMU fields")

                window_count += 1
                last_packet_time = time.monotonic()
                latest = data

                if first_seq is None:
                    first_seq = seq
                    highest_seq = seq

                # 重复包不增加唯一包数量。
                # Duplicate packets do not increase the unique count.
                if seq in seen:
                    duplicates += 1
                else:
                    if seq < highest_seq:
                        out_of_order += 1
                    seen.add(seq)
                    first_seq = min(first_seq, seq)
                    highest_seq = max(highest_seq, seq)

            except socket.timeout:
                pass
            except (ValueError, KeyError, TypeError, UnicodeError):
                invalid += 1

            now = time.monotonic()

            if now - report_time >= 1.0:
                elapsed = now - report_time
                rate = window_count / elapsed

                # 缺口仅覆盖已观察到的序号范围，迟到包可填补缺口。
                # Gaps cover only the observed sequence range; late packets can fill them.
                gaps = (
                    highest_seq - first_seq + 1 - len(seen)
                    if first_seq is not None else 0
                )

                errorTime = (
                    f"{now - last_packet_time:.1f}s"
                    if last_packet_time is not None else "N/A"
                )

                values = (
                    f"ax={latest['ax_mps2']:.2f} "
                    f"ay={latest['ay_mps2']:.2f} "
                    f"az={latest['az_mps2']:.2f} "
                    f"gx={latest['gx_radps']:.3f} "
                    f"gy={latest['gy_radps']:.3f} "
                    f"gz={latest['gz_radps']:.3f}"
                    if latest is not None else "No IMU data yet"
                )

                print(
                    f"rx={rate:.1f} Hz | unique={len(seen)} | "
                    f"gaps={gaps} | duplicate={duplicates} | "
                    f"out_of_order={out_of_order} | invalid={invalid} | "
                    f"errorTime={errorTime} | {values}",
                    flush=True,
                )

                window_count = 0
                report_time = now

    except KeyboardInterrupt:
        print("\nReceiver stopped.")