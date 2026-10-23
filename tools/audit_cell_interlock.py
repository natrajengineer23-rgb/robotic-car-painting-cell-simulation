#!/usr/bin/env python3
"""Validate an injected software shared-stop test using Webots rail records.

These results are simulation only. This cannot certify industrial functional safety,
robot swept-volume clearance, or hardware-rated stop performance.
"""
import argparse, csv, json, math, sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]

class AuditError(RuntimeError):pass

def load(root,side):
    path=root/'logs'/side/f'stage21_{side}_rail_stations.csv'
    if not path.is_file():raise AuditError(f'Missing file {path}')
    with path.open(newline='') as f:
        reader=csv.DictReader(f)
        needed={'station','status','time_s'}
        if not needed.issubset(reader.fieldnames or []):raise AuditError(f'Invalid header {path}')
        rows=list(reader)
    # A successful FULL baseline produces three PASS rows. It is not a valid
    # fault-injection result: fail closed and explain the corrective action.
    if len(rows)!=1:
        statuses=[r.get('status','').strip() for r in rows]
        if len(rows)==3 and all(v=='PASS' for v in statuses):
            raise AuditError(
                f'{side}: found three PASS stations from a normal FULL painting run, '
                'NOT an injected-stop run. Open the corresponding '
                'CarPaintingCell_Stage21_InjectStop_Far.wbt or '
                'CarPaintingCell_Stage21_InjectStop_Near.wbt, '
                'Reset and Play it to completion, then rerun this audit '
                'BEFORE running a normal Quick/Full world again. '
                '(Worlds overwrite the same rail CSV files.)'
            )
        raise AuditError(
            f'{side}: expected exactly one aborted station from the InjectStop QUICK world; '
            f'found {len(rows)} station rows with statuses {statuses!r}. '
            'Run the appropriate InjectStop world, not the normal Quick/Full world, '
            'and audit immediately after it ends.'
        )
    r=rows[0]
    if str(r['station']).strip()!='1':raise AuditError(f'{side}: wrong station {r["station"]}')
    if r['status']=='PASS':
        raise AuditError(
            f'{side}: normal Quick baseline PASS found. '
            'No injected shared-stop was recorded. '
            'Run the InjectStop world and audit immediately afterward.'
        )
    try:t=float(r['time_s'])
    except Exception as exc:raise AuditError(f'{side}: bad time_s') from exc
    if not math.isfinite(t) or t<=0:raise AuditError(f'{side}: invalid time {t}')
    if (r.get('painting_targets_passed') or '').strip():raise AuditError(f'{side}: unexpected target pass count')
    return {'side':side,'reason':r['status'],'time_s':t}

def arm_hold(root,side,rail_stop_time):
    path=root/'logs'/side/f'stage21_{side}_arm_stop.csv'
    if not path.is_file():raise AuditError(f'Missing arm-stop trace {path}')
    with path.open(newline='') as f:
        reader=csv.DictReader(f)
        if not {'stop_time_s','samples','max_joint_drift_rad','hold_status'}.issubset(reader.fieldnames or []):
            raise AuditError(f'{side}: arm stop trace headers incomplete')
        rows=list(reader)
    if len(rows)!=1:raise AuditError(f'{side}: arm stop trace must have one recorded event')
    r=rows[0]
    try:t=float(r['stop_time_s']);samples=int(r['samples']);drift=float(r['max_joint_drift_rad'])
    except Exception as exc:raise AuditError(f'{side}: invalid arm stop trace fields') from exc
    if not all(math.isfinite(v) for v in (t,drift)):
        raise AuditError(f'{side}: non-finite arm stop diagnostic')
    if r['hold_status']!='HOLD_WITHIN_DIAGNOSTIC_LIMIT' or samples!=32 or drift>0.050:
        raise AuditError(f'{side}: arm stop hold diagnostic failed: {r}')
    if abs(t-rail_stop_time)>2.0:
        raise AuditError(f'{side}: arm stop time {t} not correlated with rail event {rail_stop_time}')
    return {'stop_time_s':t, 'sampled_steps':samples,'max_joint_drift_rad':drift,
            'diagnostic_status':r['hold_status']}

def analyze(root,injected):
    peer='far' if injected=='near' else 'near'
    a=load(root,injected);b=load(root,peer)
    if a['reason']!='INJECTED_TEST_CELL_STOP_FROM_'+injected.upper():
        raise AuditError(f'Injection not recorded by {injected}: {a["reason"]}')
    if not b['reason'].startswith('SHARED_CELL_INTERLOCK CELL_STOP:'+injected+':INJECTED_TEST_CELL_STOP_FROM_'+injected.upper()):
        raise AuditError(f'Peer rail did not record shared STOP: {b["reason"]}')
    lag=b['time_s']-a['time_s']
    if not -0.10<=lag<=2.0:
        raise AuditError(f'Unexpected peer rail stop response lag={lag:.3f}s')
    arm_holds={side:arm_hold(root,side,record['time_s']) for side,record in ((injected,a),(peer,b))}
    return {'status':'SIMULATED_SHARED_STOP_AND_ARM_HOLD_DIAGNOSTICS_PASSED', 'injected_side':injected,
            'initiator':a,'peer':b,'arm_hold_diagnostics':arm_holds,'peer_record_lag_s':round(lag,3),
            'limitations':'Software stop and diagnostic joint holding in Webots only; NOT safety-rated emergency stopping, real-world response-time validation, or collision proof.'}

def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--injected-side',required=True,choices=('far','near'))
    p.add_argument('--root',type=Path,default=ROOT)
    args=p.parse_args(argv)
    try:result=analyze(args.root,args.injected_side)
    except AuditError as exc:
        print('NOT VALIDATED:',exc,file=sys.stderr);return 2
    print(json.dumps(result,indent=2))
    return 0
if __name__=='__main__':raise SystemExit(main())
