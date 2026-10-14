from pathlib import Path
from datetime import datetime
import shutil

# 保留本地网络配置，只修改 IMU 恢复逻辑。
# Preserve local network settings and change only IMU recovery logic.
p = Path(r'C:\Users\<user>\Documents\Arduino\imu_udp_50hz_test\imu_udp_50hz_test.ino')
s = p.read_text(encoding='utf-8-sig')
assert 'bool imuOnline' not in s, 'Recovery logic already present'
if '#include <esp_timer.h>' not in s:
    s = '#include <esp_timer.h>\n' + s
helpers = '''
// IMU 通信状态；不能判断具体哪根线断开。
// IMU communication state; this cannot identify an individual disconnected wire.
bool imuOnline = false;
int64_t lastImuRetryUs = 0;
int64_t lastGoodImuUs = 0;

void markImuOffline() {
  if (imuOnline) {
    Serial.println("IMU disconnected / no fresh data; waiting for connection...");
  }
  imuOnline = false;
  lastImuRetryUs = esp_timer_get_time();
}

bool reconnectImu() {
  // 重建 I2C 并限制单次总线等待，然后重新初始化传感器。
  // Recreate I2C with a bounded bus timeout, then initialize the sensor again.
  Wire.end();
  Wire.begin(21, 22);
  Wire.setClock(100000);
  Wire.setTimeOut(50);
  imu.begin(Wire, 1);
  if (imu.status != ICM_20948_Stat_Ok || !imu.isConnected()) {
    Serial.println("IMU unavailable; waiting for connection...");
    return false;
  }
  // 初始化成功后仍需在一秒内读到新数据。
  // After initialization, a fresh sample must arrive within one second.
  imuOnline = true;
  lastGoodImuUs = esp_timer_get_time();
  lastSampleUs = lastGoodImuUs;
  Serial.println("IMU connection restored; resuming sampling.");
  return true;
}

'''
s = s.replace('void setup() {', helpers + 'void setup() {', 1)
start = s.index('  // 初始化 IMU；')
end = s.index('  // 连接 Wi-Fi', start)
s = s[:start] + '''  // 启动时尝试一次；未连接时由 loop 定期重试。
  // Try once at startup; loop retries periodically if unavailable.
  reconnectImu();
  lastImuRetryUs = esp_timer_get_time();

''' + s[end:]
point = '  // 尚未到下一个采样时间'
idx = s.index(point)
s = s[:idx] + '''  // 断线时每秒尝试恢复，期间不发送旧测量。
  // Retry once per second while offline; never send stale measurements.
  if (!imuOnline) {
    if (nowUs - lastImuRetryUs >= 1000000LL) {
      reconnectImu();
      lastImuRetryUs = esp_timer_get_time();
    }
    lastSampleUs = esp_timer_get_time();
    delay(1);
    return;
  }

''' + s[idx:]
s = s.replace('''    notReadyCount++;
    return;''', '''    notReadyCount++;
    // 区分暂时未就绪与失去通信；无新数据超过一秒也触发恢复。
    // Distinguish not-ready from lost communication; a one-second stall also triggers recovery.
    if (!imu.isConnected() ||
        esp_timer_get_time() - lastGoodImuUs >= 1000000LL) {
      markImuOffline();
    }
    return;''', 1)
s = s.replace('''    readFailCount++;
    return;''', '''    readFailCount++;
    markImuOffline();
    return;''', 1)
s = s.replace('  readCount++;', '  lastGoodImuUs = esp_timer_get_time();\n  readCount++;', 1)
s = s.replace('missed_slots=%lu | wifi=%s\\n', 'missed_slots=%lu | wifi=%s | imu=%s\\n')
s = s.replace('WiFi.status() == WL_CONNECTED ? "OK" : "DOWN"\n', 'WiFi.status() == WL_CONNECTED ? "OK" : "DOWN",\n      imuOnline ? "OK" : "WAITING"\n')
# 在原目录保留备份，不输出或复制凭据到项目记录。
# Keep a backup beside the sketch without exposing credentials in project logs.
backup = p.with_suffix('.ino.' + datetime.now().strftime('%Y%m%d_%H%M%S') + '.bak')
shutil.copy2(p, backup)
p.write_text(s, encoding='utf-8')
print('Updated IMU recovery logic; backup saved beside original sketch.')
