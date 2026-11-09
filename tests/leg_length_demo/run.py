"""Host tests: real application and team PID; hardware/RTOS boundaries are stubbed."""
import argparse
from pathlib import Path
import subprocess

root = Path(__file__).resolve().parents[2]
parser = argparse.ArgumentParser()
parser.add_argument('--cc', default='gcc')
args = parser.parse_args()
output_dir = root / 'build/leg_length_demo'
output_dir.mkdir(parents=True, exist_ok=True)
common = [
    args.cc, '-std=c11', '-O2', '-Wall', '-Wextra', '-Werror',
    '-I', str(root / 'tests/leg_length_demo/stubs'),
    '-I', str(root / 'Modules/algorithm/controller'),
    '-I', str(root / 'UserApp/robot/leg_length_demo'),
    str(root / 'Modules/algorithm/controller/controller.c'),
    str(root / 'UserApp/robot/leg_length_demo/length_control.c'),
]
output = output_dir / 'test_length.exe'
subprocess.run([*common, str(root / 'tests/leg_length_demo/test_length.c'), '-lm', '-o', str(output)], check=True)
subprocess.run([str(output)], check=True)
for calibrated in (0, 1):
    output = output_dir / f'test_app_{calibrated}.exe'
    subprocess.run([*common, '-I', str(root / 'tests/leg_length_demo/app_stubs'),
                    f'-DLEG_LENGTH_CALIBRATED={calibrated}',
                    str(root / 'tests/leg_length_demo/test_app.c'), '-lm', '-o', str(output)], check=True)
    subprocess.run([str(output)], check=True)
