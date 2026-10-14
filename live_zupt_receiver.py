"""实时接收、显示并保存静止检测状态。 / Receive, display and save live stationary detection.
运行 / Run: python live_zupt_receiver.py
Ctrl+C 停止；默认持续接收。 / Ctrl+C stops; reception is continuous by default.
"""
import argparse
import base64
import hashlib
import json
import math
import socket
import time
from datetime import datetime, timezone, timedelta
from pathlib import Path
from imu_recorder import is_valid_imu
from stationary_stream import StationaryDetector

ROOT=Path(__file__).resolve().parent
MODEL=ROOT/'analysis/zupt_training_20261007_164450_519158/model.json'
LABELS={'unknown':'未知 / Unknown','confirming':'确认中 / Confirming',
        'stationary_candidate':'候选静止 / Stationary candidate',
        'not_stationary_candidate':'未满足静止条件 / Not stationary candidate'}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bind',default='0.0.0.0')
    parser.add_argument('--port',type=int,default=9000)
    parser.add_argument('--duration',type=float,default=0,help='首个有效包后秒数，0=不限 / seconds after first packet; 0=unlimited')
    parser.add_argument('--model',type=Path,default=MODEL)
    parser.add_argument('--output-root',type=Path,default=ROOT/'live_recordings')
    parser.add_argument('--notes',default='Thigh-mounted stationary preflight; not a labeled validation trial')
    args=parser.parse_args()
    if not math.isfinite(args.duration) or args.duration<0:parser.error('Invalid duration')
    model_bytes=args.model.read_bytes();model=json.loads(model_bytes)
    detector=StationaryDetector(model)
    state=detector.no_data()
    counts=dict(valid=0,invalid=0,foreign_sender=0,ack_failures=0,timeouts=0)
    start=None;last_valid=None;last_report=0;last_flush=0;last_count=0;peer=None
    reason='stopped';stale=True
    now=datetime.now(timezone(timedelta(hours=8)))
    folder=args.output_root/now.strftime('%Y%m%d_%H%M%S_%f_live')
    # 先绑定，避免端口占用时创建空记录。 / Bind before creating files to avoid empty sessions on conflicts.
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sock:
        try:sock.bind((args.bind,args.port))
        except OSError as error:
            parser.exit(1,f'无法监听 / Cannot bind UDP {args.port}: {error}\n请关闭旧接收器 / Close other receivers.\n')
        sock.settimeout(0.05)
        folder.mkdir(parents=True,exist_ok=False)
        (folder/'model.json').write_bytes(model_bytes)
        metadata=dict(created_at=now.isoformat(),mounting='thigh',notes=args.notes,
                      duration_s=args.duration,model_sha256=hashlib.sha256(model_bytes).hexdigest(),
                      confirmation_s=0.5,receive_timeout_s=0.5,
                      timestamp_note='MCU and host clocks are not synchronized. States are candidates, not displacement measurements.')
        (folder/'session.json').write_text(json.dumps(metadata,ensure_ascii=False,indent=2),encoding='utf-8')
        print('保存位置 / Output:',folder,flush=True)
        print('等待数据，先站稳；Ctrl+C 停止 / Waiting; stand still first; Ctrl+C to stop.',flush=True)
        with (folder/'packets.jsonl').open('x',encoding='utf-8') as raw_file, (folder/'states.jsonl').open('x',encoding='utf-8') as states:
            def save_state(kind, arrival, **extra):
                states.write(json.dumps(dict(event=kind,arrival_monotonic_ns=arrival,
                    elapsed_s=(arrival-start)/1e9 if start is not None else None,**extra,**state),ensure_ascii=False)+'\n')
            try:
                while True:
                    tick=time.monotonic_ns()
                    if start is not None and args.duration and (tick-start)/1e9>=args.duration:
                        reason='duration_complete';break
                    # 无论是否有探测/坏包到达，都按最后有效 IMU 包判断断流。
                    # Use last valid IMU arrival for timeout even while probes or bad packets arrive.
                    if last_valid is not None and (tick-last_valid)/1e9>=0.5 and not stale:
                        state=detector.no_data();stale=True;counts['timeouts']+=1
                        save_state('timeout',tick)
                    try:raw,sender=sock.recvfrom(65535)
                    except socket.timeout:raw=None
                    if raw is not None:
                        arrival=time.monotonic_ns();wall=time.time_ns()
                        if start is not None and args.duration and (arrival-start)/1e9>=args.duration:
                            reason='duration_complete';break
                        if raw.startswith(b'DIP_PING '):
                            token=raw[9:]
                            if len(token)==16 and all(c in b'0123456789abcdef' for c in token):
                                try:sock.sendto(b'DIP_ACK '+token,sender)
                                except OSError:counts['ack_failures']+=1
                        else:
                            try:p=json.loads(raw.decode('utf-8'));valid=is_valid_imu(p)
                            except (ValueError,UnicodeError,OverflowError):p=None;valid=False
                            foreign=peer is not None and sender!=peer
                            raw_file.write(json.dumps(dict(arrival_unix_ns=wall,arrival_monotonic_ns=arrival,
                                sender_ip=sender[0],sender_port=sender[1],valid_imu=valid,
                                raw_base64=base64.b64encode(raw).decode('ascii'),packet=p if valid else None,
                                accepted_for_detection=valid and not foreign),ensure_ascii=False)+'\n')
                            if foreign:counts['foreign_sender']+=1
                            elif not valid:
                                counts['invalid']+=1;state=detector.no_data()
                                save_state('invalid_packet',arrival)
                            else:
                                if peer is None:peer=sender
                                if start is None:
                                    start=arrival
                                    print('已收到 IMU，请保持站稳 / IMU received; remain standing still.',flush=True)
                                last_valid=arrival;stale=False;counts['valid']+=1
                                d=p['data']
                                state=detector.push(p['timestamp_us'],p['sequence_number'],
                                    [d[k] for k in ('ax_mps2','ay_mps2','az_mps2')],
                                    [d[k] for k in ('gx_radps','gy_radps','gz_radps')])
                                save_state('measurement',arrival,timestamp_us=p['timestamp_us'],sequence_number=p['sequence_number'])
                    seconds=time.monotonic()
                    if seconds-last_report>=1:
                        rate=(counts['valid']-last_count)/(seconds-last_report) if last_report else 0
                        age=(time.monotonic_ns()-last_valid)/1e9 if last_valid is not None else None
                        elapsed=(time.monotonic_ns()-start)/1e9 if start is not None else 0
                        print(f"{elapsed:6.1f}s | rx={rate:5.1f}Hz | {LABELS[state['state']]} | "
                              f"ZUPT={state['zupt_allowed']} | age={age if age is None else round(age,2)}s | saved={counts['valid']}",flush=True)
                        last_report=seconds;last_count=counts['valid']
                    if seconds-last_flush>=1:raw_file.flush();states.flush();last_flush=seconds
            except KeyboardInterrupt:reason='user_stopped'
            finally:
                # 程序停止后许可失效；保留终止事件和摘要。
                # Revoke permission on shutdown; preserve final event and summary.
                state=detector.no_data();save_state('receiver_stopped',time.monotonic_ns(),stop_reason=reason)
                (folder/'summary.json').write_text(json.dumps(dict(reason=reason,**counts),indent=2),encoding='utf-8')
    print('接收结束 / Receiver stopped. Saved:',folder)


if __name__=='__main__':main()
