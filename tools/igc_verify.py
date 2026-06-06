#!/usr/bin/env python3
"""Verify IGC G record (HMAC-SHA256 signature)."""

import argparse
import base64
import hashlib
import hmac
import os
import sys


KEY = bytes([
    0x48, 0x61, 0x73, 0x6B, 0x6F, 0x56, 0x61, 0x72,
    0x69, 0x6F, 0x48, 0x4D, 0x41, 0x43, 0x4B, 0x65,
    0x79, 0x32, 0x35, 0x36, 0x42, 0x69, 0x74, 0x73,
    0x53, 0x68, 0x61, 0x72, 0x65, 0x64, 0x21, 0x21,
])


def base64_encode_20(data: bytes) -> str:
    assert len(data) >= 20
    raw = data[:20]
    result = []
    for i in range(6):
        g = (raw[i * 3] << 16) | (raw[i * 3 + 1] << 8) | raw[i * 3 + 2]
        result.append('ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/'[(g >> 18) & 0x3F])
        result.append('ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/'[(g >> 12) & 0x3F])
        result.append('ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/'[(g >> 6) & 0x3F])
        result.append('ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/'[g & 0x3F])
    g = (raw[18] << 16) | (raw[19] << 8)
    result.append('ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/'[(g >> 18) & 0x3F])
    result.append('ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/'[(g >> 12) & 0x3F])
    result.append('ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/'[(g >> 6) & 0x3F])
    result.append('=')
    return ''.join(result)


def verify_igc(filepath: str) -> dict:
    with open(filepath, 'rb') as f:
        raw = f.read()

    # Find the G record (last line starting with 'G')
    g_idx = -1
    for i in range(len(raw) - 1, -1, -1):
        if raw[i] == ord('\n') or raw[i] == ord('\r'):
            continue
        line_start = raw.rfind(b'\n', 0, i) + 1
        line = raw[line_start:i+1]
        if line.startswith(b'G'):
            g_idx = line_start
            break

    if g_idx < 0:
        return {'status': 'ERROR', 'error': 'No G record found'}

    g_line = raw[g_idx:].splitlines()[0]
    g_value = g_line[1:].decode('ascii')

    hmac_input = raw[:g_idx]
    expected = base64_encode_20(hmac.digest(KEY, hmac_input, hashlib.sha256))

    valid = (expected == g_value)

    return {
        'status': 'OK' if valid else 'FAIL',
        'file': os.path.basename(filepath),
        'g_record': 'G' + g_value,
        'expected': 'G' + expected,
        'match': valid,
        'data_bytes': len(hmac_input),
    }


def main():
    parser = argparse.ArgumentParser(description='Verify IGC G record HMAC-SHA256 signature.')
    parser.add_argument('files', nargs='+', help='IGC file(s) to verify')
    args = parser.parse_args()

    results = []
    for f in args.files:
        if not os.path.isfile(f):
            print(f'Error: {f} not found', file=sys.stderr)
            continue
        r = verify_igc(f)
        results.append(r)

    if not results:
        return 1

    for r in results:
        if r['status'] == 'ERROR':
            print(f"  {r['file']}: ERROR - {r['error']}")
        else:
            mark = '✓' if r['match'] else '✗'
            print(f"  {mark} {r['file']}: G record {'VALID' if r['match'] else 'INVALID'}")
            if not r['match']:
                print(f"     file:     {r['g_record']}")
                print(f"     expected: {r['expected']}")

    all_ok = all(r.get('match', False) for r in results if r['status'] != 'ERROR')
    return 0 if all_ok else 1


if __name__ == '__main__':
    sys.exit(main())
