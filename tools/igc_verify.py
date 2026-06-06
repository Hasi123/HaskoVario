#!/usr/bin/env python3
"""Verify IGC G record (HMAC-SHA256 signature, hex format per spec A3.6)."""

import argparse
import hashlib
import hmac
import os
import re
import sys


KEY = bytes([
    0x48, 0x61, 0x73, 0x6B, 0x6F, 0x56, 0x61, 0x72,
    0x69, 0x6F, 0x48, 0x4D, 0x41, 0x43, 0x4B, 0x65,
    0x79, 0x32, 0x35, 0x36, 0x42, 0x69, 0x74, 0x73,
    0x53, 0x68, 0x61, 0x72, 0x65, 0x64, 0x21, 0x21,
])


def parse_g_records(raw: bytes):
    """Extract G record lines from the end of the file.
    
    Returns (hex_text, g_start) where hex_text is the concatenated hex
    content from all G record lines (without 'G' prefix or CRLF), and
    g_start is the byte offset where the first G record line begins.
    """
    lines = raw.split(b'\n')

    g_content = []
    g_start = -1

    for i in range(len(lines) - 1, -1, -1):
        line = lines[i].rstrip(b'\r\n ')
        if not line:
            continue
        if line.startswith(b'G'):
            g_content.insert(0, line[1:].decode('ascii'))
            g_start = sum(len(lines[j]) + 1 for j in range(i))
        else:
            break

    if not g_content:
        return None, -1

    return ''.join(g_content), g_start


def is_hex(s: str) -> bool:
    return bool(re.match(r'^[0-9A-Fa-f]+$', s))


def verify_igc(filepath: str) -> dict:
    with open(filepath, 'rb') as f:
        raw = f.read()

    g_text, g_start = parse_g_records(raw)
    if g_text is None:
        return {'status': 'ERROR', 'error': 'No G record found', 'file': os.path.basename(filepath)}

    if not is_hex(g_text):
        return {'status': 'ERROR', 'error': 'G record is not hex', 'file': os.path.basename(filepath)}

    hmac_input = raw[:g_start]
    digest = hmac.digest(KEY, hmac_input, hashlib.sha256)
    expected = digest.hex().upper()
    valid = (expected == g_text)

    return {
        'status': 'OK' if valid else 'FAIL',
        'file': os.path.basename(filepath),
        'g_record': g_text,
        'expected': expected,
        'match': valid,
        'data_bytes': len(hmac_input),
    }


def main():
    parser = argparse.ArgumentParser(description='Verify IGC G record HMAC-SHA256 signature.')
    parser.add_argument('files', nargs='+', help='IGC file(s) to verify')
    parser.add_argument('-v', '--verbose', action='store_true', help='Show record details')
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
            mark = '\u2713' if r['match'] else '\u2717'
            print(f"  {mark} {r['file']}: G record {'VALID' if r['match'] else 'INVALID'}")
            if args.verbose:
                print(f"     file:     G{r['g_record']}")
                print(f"     expected: G{r['expected']}")
                print(f"     data:     {r['data_bytes']} bytes")
            if not r['match']:
                print(f"     file:     G{r['g_record']}")
                print(f"     expected: G{r['expected']}")

    all_ok = all(r.get('match', False) for r in results if r['status'] != 'ERROR')
    return 0 if all_ok else 1


if __name__ == '__main__':
    sys.exit(main())
