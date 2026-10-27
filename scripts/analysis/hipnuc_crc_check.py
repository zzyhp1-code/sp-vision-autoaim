# -*- coding: utf-8 -*-
"""独立复核 CH010/HIPNUC 帧 CRC：用真实 0x91 帧验证固件修改的校验口径。"""
import argparse
from pathlib import Path
import re
import sys

POLY = 0x1021


def update_crc16(crc, data):
    for byte in data:
        crc ^= (byte << 8) & 0xFFFF
        for _ in range(8):
            crc = ((crc << 1) ^ POLY) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def frame_then_payload(frame, payload_len):
    """固件 hipnuc_crc.h 的口径：跳过偏移 4、5 的两个 CRC 字节。"""
    return update_crc16(update_crc16(0, frame[:4]), frame[6:6 + payload_len])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('sample', type=Path, help='包含 live_frame[] 的 C++ 样本文件')
    args = parser.parse_args()
    try:
        text = args.sample.read_text(encoding='utf-8')
        body = re.search(r'live_frame\[\]\s*=\s*\{(.*?)\};', text, re.S)
        if body is None:
            raise ValueError('未找到 live_frame[] 样本')
        # 忽略注释中的数字，且不把超出一个字节的值静默截断。
        body = re.sub(r'/\*.*?\*/|//[^\n]*', '', body.group(1), flags=re.S)
        frame = bytes(int(v.strip(), 0) for v in body.split(',') if v.strip())
        if len(frame) < 7 or frame[:2] != b'\x5a\xa5' or frame[6] != 0x91:
            raise ValueError('样本不是完整的 0x91 帧')
        if len(frame) != int.from_bytes(frame[2:4], 'little') + 6:
            raise ValueError('样本字节数与长度字段不一致')
    except (OSError, UnicodeError, ValueError) as error:
        print(f'错误：{error}', file=sys.stderr)
        return 1
    payload_len = frame[2] | (frame[3] << 8)
    total = payload_len + 6
    recv_le = frame[4] | (frame[5] << 8)
    recv_be = (frame[4] << 8) | frame[5]
    print('帧字节数 = %d，LEN(payload) = %d，总长 = %d' % (len(frame), payload_len, total))
    print('帧头 = %02X %02X，类型 = 0x%02X，帧内 CRC 字节 = %02X %02X'
          % (frame[0], frame[1], frame[6], frame[4], frame[5]))
    print('')
    print('固件口径 (SOF+LEN+payload，跳过 CRC 字节) = 0x%04X' % frame_then_payload(frame, payload_len))
    print('整帧含 CRC 字节                        = 0x%04X' % update_crc16(0, frame))
    print('仅 payload                             = 0x%04X' % update_crc16(0, frame[6:]))
    print('仅帧头 + payload                       = 0x%04X' % update_crc16(update_crc16(0, frame[:2]), frame[6:]))
    print('')
    print('帧内 CRC 小端解释 = 0x%04X，大端解释 = 0x%04X' % (recv_le, recv_be))
    print('')
    ok = frame_then_payload(frame, payload_len) == recv_le
    print('结论：固件新口径与小端帧内CRC %s' % ('一致 ✅' if ok else '不一致 ❌'))
    # 单字节扰动敏感性
    bad = bytearray(frame)
    bad[-1] ^= 0x01
    print('单字节扰动后新口径 = 0x%04X（应不同于 0x%04X）'
          % (frame_then_payload(bad, payload_len), recv_le))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
