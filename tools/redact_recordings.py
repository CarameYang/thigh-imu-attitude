"""发布前录制脱敏：移除接收端添加的网络字段与冗余原始字节。
Pre-publication redaction: strip receiver-added network fields and redundant raw bytes.

背景 / Why:
  录制文件由 imu_recorder.py 与 live_zupt_receiver.py 写入。每一行除测量数据外，
  还包含接收端补充的字段：sender_ip / sender_port 记录当时的局域网地址，
  arrival_unix_ns 记录电脑墙上时钟，raw_base64 是原始 UDP 字节的副本。
  Recordings contain receiver-added fields: sender_ip / sender_port (the LAN address
  at capture time), arrival_unix_ns (host wall clock) and raw_base64 (a redundant copy
  of the raw UDP bytes).

  测量的数值本身（packet 对象）一字不改。脚本只删除上述接收端字段。
  The measurements themselves (the packet object) are never modified.

用法 / Usage:
    python tools/redact_recordings.py --source _raw_backup --target .

  --source 保留原始录制的目录（默认 _raw_backup，不随仓库发布）
  --target 写入脱敏副本的目录（默认当前目录，即仓库根）

  可重复运行：对已脱敏的数据再跑一次不会改变结果。
  Idempotent: re-running over already-redacted data changes nothing.
"""
import argparse
import hashlib
import json
from pathlib import Path

# 保留的逐包字段 / per-packet fields kept
KEEP = ('arrival_monotonic_ns', 'valid_imu', 'packet', 'accepted_for_detection')

# 删除的接收端字段 / receiver-added fields removed
DROP = ('sender_ip', 'sender_port', 'arrival_unix_ns', 'raw_base64')

ROOTS = ('recordings', 'live_recordings')


def redact_packets(path, target_path):
    """逐行重写 packets.jsonl，只保留白名单字段。返回统计信息。"""
    kept_lines = []
    dropped = {name: 0 for name in DROP}
    bad = 0
    with path.open(encoding='utf-8') as handle:
        for line in handle:
            if not line.strip():
                continue
            try:
                record = json.loads(line)
            except ValueError:
                # 无法解析的行原样保留，不静默丢弃。/ keep unparsable lines verbatim
                bad += 1
                kept_lines.append(line.rstrip('\n'))
                continue
            for name in DROP:
                if name in record:
                    dropped[name] += 1
            kept_lines.append(json.dumps(
                {k: v for k, v in record.items() if k in KEEP},
                ensure_ascii=False))
    target_path.parent.mkdir(parents=True, exist_ok=True)
    target_path.write_text('\n'.join(kept_lines) + '\n', encoding='utf-8')
    return len(kept_lines), dropped, bad


def sanitize_model(path, target_path, sources_index, target_base):
    """替换 sources 中的绝对路径，并按脱敏后的文件重算 sha256。

    sources_index 把会话名映射到所属根目录；target_base 是脱敏输出的根目录。
    sources_index maps session name -> owning root; target_base is the output root.
    """
    model = json.loads(path.read_text(encoding='utf-8'))
    sources = model.get('sources')
    rewritten = 0
    if isinstance(sources, dict):
        for name, entry in sources.items():
            if not isinstance(entry, dict):
                continue
            root = sources_index.get(name, 'recordings')
            entry['path'] = f'<repo-root>/{root}/{name}'
            packets = target_base / root / name / 'packets.jsonl'
            if packets.is_file():
                entry['sha256'] = hashlib.sha256(packets.read_bytes()).hexdigest()
            rewritten += 1
    target_path.parent.mkdir(parents=True, exist_ok=True)
    target_path.write_text(json.dumps(model, ensure_ascii=False, indent=2) + '\n',
                           encoding='utf-8')
    return rewritten


def collect(source):
    """返回 [(相对路径, 绝对路径)]，按根目录与会话名排序。"""
    found = []
    for root in ROOTS:
        base = source / root
        if base.is_dir():
            found.extend(base.rglob('*'))
    return sorted(p for p in found if p.is_file())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=Path('_raw_backup'))
    parser.add_argument('--target', type=Path, default=Path('.'))
    args = parser.parse_args()
    if not args.source.is_dir():
        parser.error(f'source directory not found: {args.source}')

    files = collect(args.source)
    sources_index = {}
    for root in ROOTS:
        base = args.source / root
        if base.is_dir():
            for session in sorted(p for p in base.iterdir() if p.is_dir()):
                sources_index[session.name] = root

    totals = {name: 0 for name in DROP}
    counts = {'packets': 0, 'copied': 0, 'models': 0, 'skipped': 0}

    # 第一遍：重写所有 packets.jsonl，使哈希重算与处理顺序无关。
    # Pass 1: rewrite every packets.jsonl, so re-hashing is order-independent.
    for item in files:
        if item.name != 'packets.jsonl':
            continue
        relative = item.relative_to(args.source)
        lines, dropped, bad = redact_packets(item, args.target / relative)
        counts['packets'] += 1
        for name in DROP:
            totals[name] += dropped[name]
        note = f'  ({bad} unparsable lines kept verbatim)' if bad else ''
        print(f'{relative}: {lines} records{note}')

    # 第二遍：处理其余文件。/ Pass 2: everything else.
    for item in files:
        if item.name == 'packets.jsonl':
            continue
        relative = item.relative_to(args.source)
        target_path = args.target / relative
        if item.suffix == '.zip':
            counts['skipped'] += 1     # 归档副本不入库 / archives are not published
            continue
        if item.name == 'model.json':
            n = sanitize_model(item, target_path, sources_index, args.target)
            counts['models'] += 1
            print(f'{relative}: {n} source entries sanitised')
        else:
            target_path.parent.mkdir(parents=True, exist_ok=True)
            target_path.write_bytes(item.read_bytes())
            counts['copied'] += 1

    print('\n脱敏汇总 / Redaction summary')
    print(f"  packets.jsonl rewritten : {counts['packets']}")
    print(f"  model.json sanitised    : {counts['models']}")
    print(f"  other files copied      : {counts['copied']}")
    print(f"  archives skipped        : {counts['skipped']}")
    for name in DROP:
        print(f'  field removed  {name:<24}: {totals[name]}')
    print('  测量数值未改动 / measurement values unchanged')


if __name__ == '__main__':
    main()
