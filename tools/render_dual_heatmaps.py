#!/usr/bin/env python3
"""Compare six local relative-exposure maps using ONE common color scale.

No external dependencies. Must be run after full Stage 21 Webots run.
Output SVGs are derived from measured virtual-spray events (assumed Gaussian fan),
not photos or measured film-thickness maps.
"""
import argparse
import csv
import html
import math
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]

def load(path):
    by={}
    if not path.exists():raise FileNotFoundError(path)
    with path.open(newline='',encoding='utf8') as f:
        rows=list(csv.DictReader(f))
    for r in rows:
        st=int(r['station']);val=float(r['relative_deposition_index'])
        if not math.isfinite(val) or val<0:raise ValueError('Invalid dose in '+str(path))
        by.setdefault(st,[]).append((int(r['ix']),int(r['iz']),val,r['region']))
    if not by:raise ValueError('No dose cells in '+str(path))
    return by

def ramp(v,limit):
    t=max(0.,min(1.,v/limit)) if limit>0 else 0
    # A common interpolated dark-purple -> gold scale makes cross-side differences visible.
    stops=[(15,18,44),(70,55,125),(160,93,91),(239,191,66)]
    x=t*(len(stops)-1);idx=min(int(x),len(stops)-2);a=x-idx
    return '#'+''.join(f'{round(stops[idx][j]*(1-a)+stops[idx+1][j]*a):02x}' for j in range(3))

def svg_grid(side,st,rows,limit):
    nx=max(r[0] for r in rows)+1;nz=max(r[1] for r in rows)+1
    scale=8;w=nx*scale;h=nz*scale
    out=[f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {w} {h+42}" width="{w}" height="{h+42}">',
         f'<title>Stage 21 {html.escape(side)} station {st} relative deposition (virtual model only)</title>',
         f'<rect x="0" y="0" width="{w}" height="{h+42}" fill="#111b27"/>',
         f'<text x="8" y="20" fill="white" font-size="14" font-family="sans-serif">{side.upper()} SIDE / PATCH {st}</text>',
         '<g transform="translate(0 35)">']
    for ix,iz,v,region in rows:
        stroke='stroke="#f9f9f9" stroke-opacity=".20" stroke-width=".4"' if region=='patch' else ''
        out.append(f'<rect x="{ix*scale}" y="{(nz-1-iz)*scale}" width="{scale}" height="{scale}" fill="{ramp(v,limit)}" {stroke}/>')
    out.extend(['</g>','</svg>'])
    return '\n'.join(out)

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--root',type=Path,default=ROOT)
    a=ap.parse_args()
    try:
        data={s:load(a.root/'logs'/s/f'stage21_{s}_deposition_grid.csv') for s in ('near','far')}
    except (FileNotFoundError, ValueError) as exc:
        print(f'No heatmaps available yet: {exc}. Run Webots Stage 21 first.',file=sys.stderr)
        return 2
    allvals=[r[2] for d in data.values() for rows in d.values() for r in rows]
    limit=max(allvals)
    if limit<=0:raise SystemExit('Invalid all-zero dose maps; was virtual spray gate ever open?')
    paths=[]
    for side in ('near','far'):
        for st,rows in sorted(data[side].items()):
            name=a.root/'logs'/side/f'stage21_{side}_relative_dose_patch{st}.svg'
            name.write_text(svg_grid(side,st,rows,limit),encoding='utf8')
            paths.append(name)
    info=a.root/'logs'/'stage21_heatmap_scale.txt'
    info.write_text(f'All heatmaps share common relative maximum dose index={limit:.8f}\n'
                    'Orange/bright represents HIGHER ASSUMED RELATIVE dose, not physical thickness.\n'
                    'Outlined cells are within the local patch; the apron is a theoretical overspray-potential proxy.\n')
    for p in paths:print('Wrote',p)
    print('Wrote',info)
if __name__=='__main__':raise SystemExit(main())
