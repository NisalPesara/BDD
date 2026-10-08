#!/usr/bin/env python3
"""Split the typed stream into rectangular CSV files for plotting/analysis."""
import argparse,csv
from pathlib import Path

def split(source,out):
    out.mkdir(parents=True,exist_ok=True)
    schemas={'EVENT':['t_us','epoch','reason']}; handles={};writers={};counts={};bad=0
    try:
        with source.open(encoding='utf-8',errors='replace') as f:
            for line in f:
                line=line.strip()
                if not line:continue
                if line.startswith('# '):
                    row=next(csv.reader([line[2:]]));kind=row[0]
                    if kind in ('SAMPLE','PULSE','SPO2','STATUS','NIRS'):schemas[kind]=row[1:]
                    continue
                if line.startswith('#'):continue
                row=next(csv.reader([line]));kind=row[0]
                if kind not in schemas or len(row)-1!=len(schemas[kind]):bad+=1;continue
                if kind not in writers:
                    path=out/(kind.lower()+'.csv')
                    handles[kind]=path.open('x',encoding='utf-8',newline='')
                    writers[kind]=csv.writer(handles[kind]);writers[kind].writerow(schemas[kind])
                writers[kind].writerow(row[1:]);counts[kind]=counts.get(kind,0)+1
    finally:
        for f in handles.values():f.close()
    print('Rows:',counts,'Malformed/unknown records skipped:',bad)
    return counts,bad
if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('source',type=Path);ap.add_argument('--out',type=Path,required=True)
    args=ap.parse_args();split(args.source,args.out)
