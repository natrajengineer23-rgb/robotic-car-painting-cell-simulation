#!/usr/bin/env python3
"""Validate and compare two independently logged Stage 21 robot/rail painting sides.

NO hardcoded results. Fail closed when any side's generated logs are missing,
inconsistent, incomplete, non-finite, or paint-dose instrumentation is absent.
All quality is model-derived relative exposure, NOT physical coating thickness.
"""
from __future__ import annotations
import argparse
import csv
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
REQUIRED_COVERAGE = ('station', 'pose_passes', 'valid_virtual_spray_samples',
                     'covered_cells', 'total_cells', 'local_patch_coverage_percent')
REQUIRED_DOSE = ('station', 'pose_passes', 'dose_samples', 'patch_cells',
                 'dose_cv', 'relative_20pct_coverage_percent',
                 'overspray_apron_fraction_percent', 'dose_mean_index')

class ValidationError(RuntimeError):
    pass

def load_rows(path: Path, required: tuple[str, ...]) -> list[dict[str,str]]:
    if not path.is_file():
        raise ValidationError(f'Missing Webots output: {path}')
    with path.open(newline='', encoding='utf8') as f:
        r = csv.DictReader(f)
        missing = set(required) - set(r.fieldnames or ())
        if missing: raise ValidationError(f'{path.name} missing columns: {sorted(missing)}')
        rows = list(r)
    if not rows: raise ValidationError(f'Empty Webots output: {path}')
    return rows

def finite_value(row: dict[str,str], name: str, origin: str) -> float:
    try:
        n = float(row[name])
    except (ValueError, TypeError, KeyError) as exc:
        raise ValidationError(f'{origin}: invalid {name}: {row.get(name)!r}') from exc
    if not math.isfinite(n):
        raise ValidationError(f'{origin}: nonfinite {name}: {n}')
    return n

def integer(row: dict[str,str], name: str, origin: str) -> int:
    x=finite_value(row,name,origin)
    if x<0 or not x.is_integer():
        raise ValidationError(f'{origin}: invalid count {name}={x}')
    return int(x)

def check_side(root: Path, side: str, expected_stations: int) -> dict:
    folder=root/'logs'/side
    p=lambda name: folder/f'stage21_{side}_{name}.csv'
    rail=load_rows(p('rail_stations'),('station','status','painting_targets_passed','time_s'))
    poses=load_rows(p('control_waypoint_summary'),('station','waypoint','status','xyz_error_m','aim_error_deg'))
    cov=load_rows(p('coverage_summary'),REQUIRED_COVERAGE)
    dose=load_rows(p('deposition_summary'),REQUIRED_DOSE)
    expected=expected_stations*15
    for name, rows in [('rail',rail),('coverage',cov),('dose',dose)]:
        ids=[integer(r,'station',name) for r in rows]
        if ids!=list(range(1,expected_stations+1)):
            raise ValidationError(f'{side} {name}: stations {ids}, expected 1..{expected_stations} exactly')
    if len(poses)!=expected:
        raise ValidationError(f'{side}: {len(poses)} pose summaries, expected {expected}')
    seen=set()
    for r in poses:
        key=integer(r,'waypoint','pose')
        if key in seen or key not in range(1,expected+1):
            raise ValidationError(f'{side}: duplicate or out-of-bounds waypoint {key}')
        seen.add(key)
        if r['status']!='POSE_PASS':
            raise ValidationError(f'{side}: waypoint {key} is {r["status"]}, not pose-qualified')
        xyz=finite_value(r,'xyz_error_m','pose')
        ang=finite_value(r,'aim_error_deg','pose')
        if xyz>0.0450001 or ang>12.0001:
            raise ValidationError(f'{side}: pose {key} exceeds geometric/aim tolerances')
    cov_stats=[];dose_stats=[]
    for n in range(expected_stations):
        rr,cc,dd=rail[n],cov[n],dose[n]
        if rr['status']!='PASS' or integer(rr,'painting_targets_passed','rail')!=15:
            raise ValidationError(f'{side}: rail station {n+1} not 15/15 PASS')
        if integer(cc,'pose_passes','coverage')!=15 or integer(dd,'pose_passes','dose')!=15:
            raise ValidationError(f'{side}: coverage/dose station {n+1} pose mismatch')
        samples_c=integer(cc,'valid_virtual_spray_samples','coverage')
        samples_d=integer(dd,'dose_samples','dose')
        if samples_c!=samples_d or samples_d==0:
            raise ValidationError(f'{side}: station {n+1} spray-credit mismatch/zero: {samples_c}/{samples_d}')
        covered=integer(cc,'covered_cells','coverage')
        cells=integer(cc,'total_cells','coverage')
        patch_cells=integer(dd,'patch_cells','dose')
        if cells!=340 or patch_cells!=340 or covered>cells:
            raise ValidationError(f'{side}: station {n+1} patch grid invalid')
        percent=finite_value(cc,'local_patch_coverage_percent','coverage')
        if not (0<=percent<=100 and abs(percent-100*covered/cells)<0.05):
            raise ValidationError(f'{side}: station {n+1} coverage percentage inconsistent')
        cv=finite_value(dd,'dose_cv','dose')
        mean=finite_value(dd,'dose_mean_index','dose')
        apr=finite_value(dd,'overspray_apron_fraction_percent','dose')
        local20=finite_value(dd,'relative_20pct_coverage_percent','dose')
        if not (cv>=0 and mean>0 and 0<=apr<=100 and 0<=local20<=100):
            raise ValidationError(f'{side}: station {n+1} modeled dose metrics out of range')
        cov_stats.append(percent)
        dose_stats.append({'dose_cv':cv, 'overspray_apron_fraction_percent':apr,
                           'relative_20pct_coverage_percent':local20,
                           'mean_relative_dose':mean, 'valid_samples':samples_d,
                           'virtual_coverage_percent':percent})
    end_t=finite_value(rail[-1],'time_s','rail')
    if end_t<=0:raise ValidationError(f'{side}: nonpositive completion time')
    return {
        'side':side,'stations':expected_stations,'passed_poses':expected,
        'completion_sim_time_s':end_t,
        'minimum_virtual_footprint_coverage_percent':min(cov_stats),
        'patches':dose_stats,
    }

def analyze(root: Path, mode: str) -> dict:
    n=1 if mode=='quick' else 3
    side_data={s:check_side(root,s,n) for s in ('near','far')}
    near=side_data['near']['completion_sim_time_s']
    far=side_data['far']['completion_sim_time_s']
    parallel=max(near,far)
    ideal_seq=near+far
    return {
        'stage':20,'mode':mode,'status':'ALL_RECORDED_TESTS_PASSED',
        'model_caveat':'Assumed relative virtual coating exposure, NOT physical deposition or certified quality.',
        'speedup_caveat':'Same-run ideal sequential estimate, NOT measured sequential baseline or factory throughput.',
        'near':side_data['near'],'far':side_data['far'],
        'both_sides_confirmed_raster_poses':2*n*15,
        'dual_parallel_simulated_completion_s':parallel,
        'same_run_ideal_sequential_estimate_s':ideal_seq,
        'same_run_ideal_side_cycle_ratio':ideal_seq/parallel,
    }

def main(argv=None) -> int:
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--mode',choices=['quick','full'],required=True)
    ap.add_argument('--root',type=Path,default=ROOT,help='Test/demo alternative project root')
    args=ap.parse_args(argv)
    try:r=analyze(args.root,args.mode)
    except ValidationError as e:
        print('NOT VALIDATED:',e,file=sys.stderr)
        return 2
    logs=args.root/'logs';logs.mkdir(parents=True,exist_ok=True)
    with (logs/'stage21_combined_quality.json').open('w',encoding='utf8') as f:
        json.dump(r,f,indent=2)
    with (logs/'stage21_combined_quality.csv').open('w',newline='',encoding='utf8') as f:
        wr=csv.writer(f)
        wr.writerow(['side','station','pose_passes','valid_model_samples','virtual_footprint_percent',
                     'relative_dose_mean','dose_cv','relative_20pct_exposure_percent',
                     'apron_proxy_percent','side_completion_sim_time_s'])
        for side in ('near','far'):
            val=r[side]
            for i,patch in enumerate(val['patches'],1):
                wr.writerow([side,i,15,patch['valid_samples'],patch['virtual_coverage_percent'],
                             patch['mean_relative_dose'],patch['dose_cv'],
                             patch['relative_20pct_coverage_percent'],
                             patch['overspray_apron_fraction_percent'],
                             val['completion_sim_time_s']])
    print(f'STAGE 20 {args.mode.upper()}: both sides verified {r["both_sides_confirmed_raster_poses"]} raster poses')
    print(f'Near last station: {r["near"]["completion_sim_time_s"]:.3f} simulated s; far: {r["far"]["completion_sim_time_s"]:.3f} simulated s')
    print(f'Dual-run makespan: {r["dual_parallel_simulated_completion_s"]:.3f} simulated s')
    print(f'Same-run ideal sequential estimate: {r["same_run_ideal_sequential_estimate_s"]:.3f} simulated s')
    print(f'Ideal side-cycle ratio: {r["same_run_ideal_side_cycle_ratio"]:.2f}× — NOT a measured baseline')
    for side in ('near','far'):
        for i,patch in enumerate(r[side]['patches'],1):
            print(f'{side} patch {i}: virtual cover={patch["virtual_coverage_percent"]:.1f}%, relative CV={patch["dose_cv"]:.3f}, apron={patch["overspray_apron_fraction_percent"]:.1f}%')
    print('REMINDER: relative deposition model ≠ measured wet-film thickness, mass or transfer efficiency.')
    print('Wrote',logs/'stage21_combined_quality.json')
    return 0

if __name__=='__main__':raise SystemExit(main())
