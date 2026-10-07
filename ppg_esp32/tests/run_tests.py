#!/usr/bin/env python3
"""Run the portable DSP and mocked-firmware tests. Requires g++ on PATH."""
from pathlib import Path
import argparse, os, subprocess, tempfile
p=argparse.ArgumentParser();p.add_argument('--sanitize',action='store_true');args=p.parse_args()
root=Path(__file__).resolve().parents[1]
flags=['-std=c++11','-Wall','-Wextra','-Wpedantic','-Werror']
flags+=['-O1','-g','-fsanitize=address,undefined','-fno-omit-frame-pointer'] if args.sanitize else ['-O2']
with tempfile.TemporaryDirectory(prefix='ppg_tests_') as d:
 for name in ('test_core','test_firmware'):
  exe=Path(d)/(name+('.exe' if os.name=='nt' else ''))
  cmd=['g++',*flags,'-I'+str(root)]
  if name=='test_firmware':cmd+=['-I'+str(root/'tests/stubs')]
  subprocess.run([*cmd,str(root/'tests'/f'{name}.cpp'),'-o',str(exe)],check=True)
  subprocess.run([str(exe)],check=True)
 # Compile feature combinations that cannot share the default core expectations.
 for options in [ ['-DNUM_ACCEL=0','-DUSE_FSR=0'], ['-DNUM_ACCEL=1','-DUSE_ANC=1'],
                  ['-DNUM_ACCEL=2','-DUSE_ANC=1','-DNIRS_MODE=1','-DCONTACT_FSR_INDEX=0'] ]:
  subprocess.run(['g++',*flags,*options,'-I'+str(root),'-I'+str(root/'tests/stubs'),
    '-c',str(root/'tests/test_firmware.cpp'),'-o',str(Path(d)/'variant.o')],check=True)
 print('PASS: compile variants (0/1/2 accelerometers, ANC, FSR contact, analog NIRS)')
