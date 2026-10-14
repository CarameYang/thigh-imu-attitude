import socket

# 监听电脑所有网卡的 UDP 9000 端口。
# Listen on UDP port 9000 on all network interfaces.
with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
    sock.bind(("0.0.0.0", 9000))
    sock.settimeout(5)
    print("Listening on UDP port 9000. Press Ctrl+C to stop.", flush=True)

    try:
        while True:
            try:
                # 接收数据，并取得发送设备的地址。
                # Receive data and obtain the sender's address.
                data, sender = sock.recvfrom(4096)
                message = data.decode("utf-8", errors="replace")
                print(f"{sender[0]}:{sender[1]} -> {message}", flush=True)
            except socket.timeout:
                # 超时只提示等待，不结束接收。
                # Report a timeout and continue listening.
                print("Waiting for ESP32...", flush=True)
    except KeyboardInterrupt:
        # 按 Ctrl+C 后正常退出并关闭套接字。
        # Exit cleanly and close the socket after Ctrl+C.
        print("\nReceiver stopped.")