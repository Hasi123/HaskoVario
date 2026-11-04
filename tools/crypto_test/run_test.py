#!/usr/bin/env python3
"""Build libraries/igcrypto for the PC and check it against hashlib/hmac.

Runs the test with several IGC_SHA256_ROUNDS_PER_STEP values, since the
digest must not depend on how the SHA-256 rounds are split, then checks
the generated IGC file with tools/igc_verify.py.

Usage: python3 tools/crypto_test/run_test.py   (needs g++)
"""

import hashlib
import hmac
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, '..', '..'))
LIBS = os.path.join(ROOT, 'libraries')
SOURCES = [
    os.path.join(HERE, 'test_igcrypto.cpp'),
    os.path.join(LIBS, 'igcrypto', 'sha256.cpp'),
    os.path.join(LIBS, 'igcrypto', 'hmac_sha256.cpp'),
    os.path.join(LIBS, 'GpsSentences', 'IGCSentence.cpp'),
]
ROUNDS = [1, 4, 7, 64]


def check_output(out: str) -> int:
    count = 0
    for line in out.splitlines():
        f = line.split(' ')
        if f[0] == 'SHA':
            msg = bytes.fromhex(f[1]) if len(f) == 3 else b''
            got = f[-1]
            exp = hashlib.sha256(msg).hexdigest()
        elif f[0] == 'HMAC':
            key, msg, got = bytes.fromhex(f[1]), bytes.fromhex(f[2]), f[3]
            exp = hmac.digest(key, msg, hashlib.sha256).hex()
        elif f[0] == 'SHA_A1M':
            got, exp = f[1], hashlib.sha256(b'a' * 1000000).hexdigest()
        else:
            raise SystemExit(f'unexpected output: {line[:80]}')
        if got != exp:
            raise SystemExit(f'MISMATCH {f[0]}: got {got} expected {exp}')
        count += 1
    return count


def main() -> int:
    with tempfile.TemporaryDirectory() as tmp:
        for rounds in ROUNDS:
            exe = os.path.join(tmp, f'test_{rounds}')
            cmd = ['g++', '-std=c++11', '-O1', '-g', '-Wall', '-Wextra', '-Wno-unused-parameter',
                   '-fsanitize=address,undefined', '-fno-sanitize-recover',
                   f'-DTEST_ROUNDS_PER_STEP={rounds}',
                   '-I', os.path.join(HERE, 'stubs'),
                   '-I', os.path.join(LIBS, 'igcrypto'),
                   '-I', os.path.join(LIBS, 'GpsSentences'),
                   '-o', exe] + SOURCES
            subprocess.run(cmd, check=True)
            run = subprocess.run([exe], cwd=tmp, check=True, capture_output=True, text=True)
            n = check_output(run.stdout)
            print(f'rounds/step {rounds:2}: {n} digests match hashlib/hmac')

            verify = subprocess.run([sys.executable, os.path.join(ROOT, 'tools', 'igc_verify.py'),
                                     os.path.join(tmp, 'test.IGC')],
                                    capture_output=True, text=True)
            print('  ' + verify.stdout.strip())
            if verify.returncode != 0:
                return 1
    print('ALL OK')
    return 0


if __name__ == '__main__':
    sys.exit(main())
