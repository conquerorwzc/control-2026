"""Host numeric tests; links the real team PID implementation."""
import argparse
from pathlib import Path
import subprocess

root = Path(__file__).resolve().parents[2]
parser = argparse.ArgumentParser()
parser.add_argument('--cc', default='gcc')
args = parser.parse_args()
output = root / 'build/leg_length_demo/test_length.exe'
output.parent.mkdir(parents=True, exist_ok=True)
subprocess.run([
    args.cc, '-std=c11', '-O2', '-Wall', '-Wextra', '-Werror',
    '-I', str(root / 'tests/leg_length_demo/stubs'),
    '-I', str(root / 'Modules/algorithm/controller'),
    '-I', str(root / 'UserApp/robot/leg_length_demo'),
    str(root / 'tests/leg_length_demo/test_length.c'),
    str(root / 'Modules/algorithm/controller/controller.c'),
    str(root / 'UserApp/robot/leg_length_demo/length_control.c'),
    '-lm', '-o', str(output),
], check=True)
subprocess.run([str(output)], check=True)
