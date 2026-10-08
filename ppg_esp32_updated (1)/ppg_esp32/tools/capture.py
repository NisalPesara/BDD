#!/usr/bin/env python3
"""Capture all firmware records without altering their raw CSV bytes."""
import argparse, threading, sys
from pathlib import Path

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--port',required=True)
    ap.add_argument('--output',type=Path,required=True);ap.add_argument('--baud',type=int,default=460800)
    args=ap.parse_args()
    try: import serial
    except ImportError: raise SystemExit('Install pyserial: python -m pip install pyserial')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    # Exclusive creation protects an earlier recording from accidental overwrite.
    with args.output.open('x',encoding='utf-8',newline='') as output, serial.Serial(args.port,args.baud,timeout=.5) as port:
        port.write(b'h\n')
        stop=threading.Event()
        def commands():
            for command in sys.stdin:
                if stop.is_set():return
                cmd=command.strip().lower()
                if cmd in ('b','r','h'):
                    try:port.write((cmd+'\n').encode())
                    except serial.SerialException:return
        threading.Thread(target=commands,daemon=True).start()
        print(f'Recording to {args.output}. Enter b/r/h for baseline/reset/headers; Ctrl+C stops.')
        try:
            while True:
                line=port.readline().decode('utf-8',errors='replace')
                if not line:continue
                output.write(line)
                if line.startswith(('STATUS,','EVENT,','#')):
                    print(line.rstrip()); output.flush()
        except KeyboardInterrupt: pass
        finally: stop.set();output.flush()
if __name__=='__main__':main()
