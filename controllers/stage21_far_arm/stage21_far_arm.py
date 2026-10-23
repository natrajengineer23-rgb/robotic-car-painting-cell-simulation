"""Stage 17 - guarded motorized rail, measured nozzle direction, optional direction-aware IK.

Two GPS receivers on the tool are used to form an ACTUAL nozzle axis:
  axis = normalize(GPS_tip - GPS_axis_base)
AUDIT: stable Stage 10 XYZ-only servo with measured angular error.
CONTROL: 6D damped least squares (XYZ + weighted unit-direction); attempts a 12deg aim tolerance.
Neither mode proves full robot-link clearance or physical spray deposition.
"""
from controller import Supervisor
from link_monitor import LinkMonitor, STOP_PROXY_M, WARN_PROXY_M
from raster_coverage import (RASTER, WAYPOINTS_PER_STATION, CoverageGrid,
                             point_segment_gap_xz, PATH_TUBE_M)
from deposition_quality import DepositionMap, DT_S
from surface_paint import SurfacePainter
from pathlib import Path
import csv,math,sys
MODE = "CONTROL"
STEP=32
JOINTS=('shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint')
HOME=[0.,-1.10,1.70,-1.90,-1.57,0.]
STATION_X=(2.10,0.55,-1.00)
LOCAL_XZ=tuple((dx,z) for dx,z,_row in RASTER)
VEHICLE_SIDE=.95
STANDOFF=.25
XYZ_TOL=.045
ANGLE_TOL_DEG=12.0
AIM=(0.,-1.,0.)  # far side: nozzle points toward -Y
AXIS_SCALE_M=.30   # weighted unit direction error enters IK as metres-equivalent
PROBE_RAD=.060
DAMPING=.085
MAX_DQ=.12
MAX_UPDATES=32 if MODE=='CONTROL' else 14
HOLD_STEPS=18
RECALIBRATE_EVERY=3 if MODE=='CONTROL' else 4
MAX_HOME_STEPS=300
CAR_BOXES=(
    ((-2.40,-.95,.51),(2.40,.95,1.13)),
    ((-1.415,-.875,1.15),(.935,.875,1.60)),
)
MIN_TIP_GAP=.10


def distance(a,b):return math.sqrt(sum((u-v)**2 for u,v in zip(a,b)))
def point_box_gap(p,lo,hi):return math.sqrt(sum(max(lo[i]-p[i],0,p[i]-hi[i])**2 for i in range(3)))
def tip_gap(p):return min(point_box_gap(p,a,b) for a,b in CAR_BOXES)
def vec_axis(base,tip):
    d=[tip[i]-base[i] for i in range(3)]
    length=math.sqrt(sum(v*v for v in d))
    if not .075<length<.14:raise RuntimeError('Invalid dual-GPS axis separation %.4f m'%length)
    return [v/length for v in d]
def aim_angle_deg(axis):
    dot=max(-1.,min(1.,sum(a*b for a,b in zip(axis,AIM))))
    return math.degrees(math.acos(dot))
def feature(p,a,mode):
    return list(p)+([AXIS_SCALE_M*v for v in a] if mode=='CONTROL' else [])
def solve_linear(a,b):
    n=len(b);m=[list(a[i])+[b[i]] for i in range(n)]
    for i in range(n):
        k=max(range(i,n),key=lambda r:abs(m[r][i]))
        m[i],m[k]=m[k],m[i]
        if abs(m[i][i])<1e-11:raise RuntimeError('Singular regularized IK matrix')
        v=m[i][i]
        for j in range(i,n+1):m[i][j]/=v
        for k in range(n):
            if k==i:continue
            mul=m[k][i]
            for j in range(i,n+1):m[k][j]-=mul*m[i][j]
    return [m[i][n] for i in range(n)]
def dls(J,error):
    rows=len(error)
    A=[[sum(J[r][j]*J[c][j] for j in range(6))+(DAMPING**2 if r==c else 0.) for c in range(rows)] for r in range(rows)]
    v=solve_linear(A,error)
    dq=[sum(J[r][j]*v[r] for r in range(rows)) for j in range(6)]
    scale=min(1.,MAX_DQ/max(max(abs(z) for z in dq),1e-9))
    return [z*scale for z in dq]
def radio_text(rx):
    f=getattr(rx,'getString',None)
    data=f() if callable(f) else rx.getData()
    return (data.decode('utf-8','replace') if isinstance(data,(bytes,bytearray)) else data).strip('\x00')

def main():
    bot=Supervisor()
    motors=[];encoders=[]
    for name in JOINTS:
        m=bot.getDevice(name)
        if m is None:raise RuntimeError('Missing motor '+name)
        m.setVelocity(.55)
        s=m.getPositionSensor()
        if s is None:raise RuntimeError('Missing motor encoder '+name)
        s.enable(STEP);motors.append(m);encoders.append(s)
    tip=bot.getDevice('spray_tip_gps')
    base=bot.getDevice('spray_axis_base_gps')
    tx=bot.getDevice('arm_to_rail');rx=bot.getDevice('rail_to_arm')
    if any(v is None for v in (tip,base,tx,rx)):
        raise RuntimeError('Missing one or more of GPS-tip, GPS-axis-base, transmitter, receiver')
    tip.enable(STEP);base.enable(STEP);rx.enable(STEP)
    logs=Path(__file__).resolve().parents[2]/'logs'/'far';logs.mkdir(parents=True,exist_ok=True)
    raw=logs/f'stage21_far_{MODE.lower()}_pose_tracking.csv'
    summary=logs/f'stage21_far_{MODE.lower()}_waypoint_summary.csv'
    raster_samples=logs/'stage21_far_virtual_spray_samples.csv'
    raster_summary=logs/'stage21_far_coverage_summary.csv'
    deposition_summary=logs/'stage21_far_deposition_summary.csv'
    deposition_grid=logs/'stage21_far_deposition_grid.csv'
    root_children=bot.getRoot().getField('children')
    paint_visual=SurfacePainter(root_children,logs,'far')
    print('[Stage 22 far paint] Visible, quality-gated BLUE coating; no paint-fluid physics.')
    linkmon=LinkMonitor(bot,motors,tip,base,logs/'stage21_far_link_clearance_trace.csv',robot_def='PAINT_ROBOT_FAR')
    sample_steps=[0]
    active_sweep=[None]  # sampled only while commanding the connected raster segment
    raster_sampling=[None]  # per-station validated GPS virtual-footprint callback
    monitor_finished=[False]
    guard_stopped=[False]
    print('[Stage 21 far arm] MODE:',MODE)
    print('[Stage 21 far raster] 15 serpentine poses/patch; 3 rows x 5 columns; virtual footprint AND assumed relative-dose model only.')
    print('[Stage 21 far raster] Spray credit is quality-gated and disabled during homing or IK calibration.')
    print('[Stage 21 far arm] Dual GPS spray-axis measurement; desired spray direction = -Y.')
    print('[Stage 21 far arm] CONTROL mode uses coupled XYZ+direction IK; AUDIT mode uses verified XYZ-only IK.')
    print('[Stage 21 far arm] Simplified tip AABB + ACTIVE robot-link proxy guard; NOT full robot-link collision clearance.')
    print('[Stage 21 far arm] Rail remains physically motorized with station drift guard.')
    def send(s):tx.send(s.encode('utf-8'))
    def guard_emergency(reason):
        # Simulation safeguard: freeze commanded joints, notify rail, then hold.
        # This is NOT a hardware-rated stop or complete swept-mesh collision protection.
        if guard_stopped[0]:raise SystemExit
        guard_stopped[0]=True
        reason=str(reason).replace(':','_')
        print('[Stage 21 far link] PROXY SAFETY STOP:',reason)
        try:
            freeze=[sensor.getValue() for sensor in encoders]
            for motor,angle in zip(motors,freeze):motor.setPosition(angle)
        except Exception as exc:print('[Stage 21 far link] WARNING: joint hold error',exc)
        try:send('STOP:PROXY_GUARD_'+reason)
        except Exception as exc:print('[Stage 21 far link] WARNING: stop radio error',exc)
        if not monitor_finished[0]:
            linkmon.summary();monitor_finished[0]=True
        # Diagnostic ONLY: check that commanded joint targets remain approximately held
        # for 1.024 simulated seconds after software STOP. NOT a safety-rated check.
        max_joint_drift=float('inf')
        count=0
        status='HOLD_UNVERIFIED'
        if 'freeze' in locals() and len(freeze)==6 and all(math.isfinite(q) for q in freeze):
            max_joint_drift=0.0
            valid=True
            for _ in range(32):
                if bot.step(STEP)==-1: break
                measured=[sensor.getValue() for sensor in encoders]
                if not all(math.isfinite(q) for q in measured):
                    valid=False;break
                max_joint_drift=max(max_joint_drift,*(abs(q-q0) for q,q0 in zip(measured,freeze)))
                count+=1
            status='HOLD_WITHIN_DIAGNOSTIC_LIMIT' if valid and count==32 and max_joint_drift<=0.050 else 'HOLD_UNVERIFIED'
        try:
            stop_trace=logs/'stage21_far_arm_stop.csv'
            with stop_trace.open('w',newline='') as sfp:
                writer=csv.writer(sfp)
                writer.writerow(['stop_time_s','reason','samples','max_joint_drift_rad','hold_status'])
                writer.writerow([round(bot.getTime()-(count*STEP/1000.0),3),reason,count,
                                 round(max_joint_drift,6) if math.isfinite(max_joint_drift) else 'NaN',status])
            print('[Stage 21 far arm] STOP HOLD DIAGNOSTIC: status=%s, samples=%d, max_joint_drift=%.5frad'%(status,count,max_joint_drift))
        except Exception as exc:
            print('[Stage 21 far arm] WARNING: failed to write stop-hold trace:',exc)
        # Remain in hold loop so the controller retains its joint position commands.
        while bot.step(STEP)!=-1:
            pass
        raise SystemExit
    if linkmon.status!='PROXY_GUARD_ACTIVE':
        guard_emergency('LINK_MONITOR_UNAVAILABLE_AT_START')
    print(f'[Stage 21 far link] PROXY GUARD ARMED: warning={WARN_PROXY_M:.3f}m stop={STOP_PROXY_M:.3f}m; checks every 3 x {STEP}ms.')
    pending=[]
    def poll():
        while rx.getQueueLength():
            data=radio_text(rx);rx.nextPacket()
            if data.startswith('STOP:'):guard_emergency('RAIL_STOP_'+data)
            pending.append(data)
    def tick(n=1):
        for _ in range(n):
            if bot.step(STEP)==-1:raise SystemExit
            poll()
            sample_steps[0]+=1
            if not monitor_finished[0] and sample_steps[0]%3==0:
                linkmon.observe()
                if linkmon.status!='PROXY_GUARD_ACTIVE' or linkmon.guard_state=='INVALID':
                    guard_emergency('LINK_MONITOR_LOST')
                if linkmon.guard_state=='STOP':
                    guard_emergency(f'PROXY_CLEARANCE_{linkmon.last_clearance:.4f}m_{linkmon.last_segment}')
                if linkmon.guard_state=='WARNING' and linkmon.warning_count%40==1:
                    print(f'[Stage 21 far link] PROXY WARNING: clearance={linkmon.last_clearance:.4f}m segment={linkmon.last_segment}')
            # Measured virtual spraying is allowed ONLY during commanded raster motion.
            # A successful Cartesian setpoint is required; no real fluid is simulated.
            if active_sweep[0] is not None and raster_sampling[0] is not None:
                try:
                    a0,b0=active_sweep[0]
                    measured_tip=list(tip.getValues())
                    measured_axis=vec_axis(base.getValues(),measured_tip)
                    ang=aim_angle_deg(measured_axis)
                    stand=measured_tip[1]-VEHICLE_SIDE
                    tube_gap=point_segment_gap_xz(measured_tip[0],measured_tip[2],a0[0],a0[2],b0[0],b0[2])
                    # Fail-closed spray gating, independent of proxy stop detection.
                    quality=(abs(stand-STANDOFF)<=0.035 and ang<=ANGLE_TOL_DEG and
                             tube_gap<=PATH_TUBE_M and tip_gap(measured_tip)>=MIN_TIP_GAP and
                             linkmon.status=='PROXY_GUARD_ACTIVE' and
                             linkmon.guard_state=='OK')
                    raster_sampling[0](bot.getTime(),measured_tip,ang,stand,tube_gap,quality)
                except (ValueError,RuntimeError) as exc:
                    active_sweep[0]=None
                    print('[Stage 21 far raster] Spray disabled after invalid sensor reading:',exc)
    def read():
        p=list(tip.getValues());a=vec_axis(base.getValues(),p)
        return p,a,[s.getValue() for s in encoders]
    def command(q):
        for m,v in zip(motors,q):m.setPosition(v)
    def check_tip(p):
        gap=tip_gap(p)
        if gap<MIN_TIP_GAP:raise RuntimeError('Tool tip violates body clearance: %.3fm'%gap)
        return gap
    def home():
        command(HOME)
        for _ in range(MAX_HOME_STEPS):
            tick()
            _,_,q=read()
            if max(abs(qj-hj) for qj,hj in zip(q,HOME))<.05:break
        else:raise RuntimeError('Robot homing timeout')
        tick(10)
    def jacobian():
        _,_,q0=read()
        command(q0);tick(HOLD_STEPS)
        p0,a0,_=read();f0=feature(p0,a0,MODE)
        J=[[0.]*6 for _ in range(len(f0))]
        for j in range(6):
            q=q0[:];q[j]+=(PROBE_RAD)
            command(q);tick(HOLD_STEPS)
            p,a,qa=read();check_tip(p)
            delta=qa[j]-q0[j]
            if abs(delta)>.005:
                fv=feature(p,a,MODE)
                for k in range(len(f0)):
                    J[k][j]=(fv[k]-f0[k])/delta
            command(q0);tick(HOLD_STEPS)
        return J

    command(HOME);tick(70)
    try:home()
    except RuntimeError as exc:
        print('[Stage 21 far arm] PREFLIGHT HOMING FAILED',exc)
        send('FAIL:0:PREFLIGHT_HOMING');return
    print('[Stage 21 far arm] ARM_HOME_VERIFIED; sending ARM_READY')
    last_ready=-1e10;completed=set()
    with raw.open('w',newline='') as fp,summary.open('w',newline='') as sf, \
         raster_samples.open('w',newline='') as rf, raster_summary.open('w',newline='') as cf, \
         deposition_summary.open('w',newline='') as dpf, deposition_grid.open('w',newline='') as dgf:
        wr=csv.writer(fp);sw=csv.writer(sf);rw=csv.writer(rf);cw=csv.writer(cf)
        dpw=csv.writer(dpf);dgw=csv.writer(dgf)
        rw.writerow(['time_s','station','raster_target','sample_x','sample_y','sample_z','axis_error_deg','standoff_m','path_tube_error_m','spray_gate_open','new_covered_cells'])
        cw.writerow(['station','raster_targets','pose_passes','valid_virtual_spray_samples','virtual_footprint_events_adding_area','covered_cells','total_cells','local_patch_coverage_percent','coverage_model'])
        dpw.writerow(['station','pose_passes','dose_samples','patch_cells','dose_mean_index','dose_std_index','dose_cv','dose_p10_index','dose_p90_index','dose_min_index','dose_max_index','relative_20pct_coverage_percent','overspray_apron_fraction_percent','inside_dose_sum','apron_dose_sum'])
        dgw.writerow(['station','ix','iz','world_x_m','world_z_m','region','relative_deposition_index'])
        wr.writerow(['time_s','station','waypoint','iteration','mode','target_x','target_y','target_z','measured_x','measured_y','measured_z','xyz_error_m','aim_error_deg','axis_x','axis_y','axis_z','standoff_m','tip_gap_m',*JOINTS])
        sw.writerow(['station','waypoint','mode','status','xyz_error_m','aim_error_deg','standoff_m','target_x','target_y','target_z'])
        while True:
            tick()
            if not completed and bot.getTime()-last_ready>=.5:
                send('ARM_READY');last_ready=bot.getTime()
            events=pending[:];pending.clear()
            for msg in events:
                if not msg.startswith('STATION:'):continue
                try:station=int(msg.split(':')[1])
                except (IndexError,ValueError):continue
                if station not in range(len(STATION_X)) or station in completed:continue
                print('[Stage 21 far arm] START station',station+1,'X=',STATION_X[station])
                send(f'STARTED:{station}')
                passed=0
                grid=CoverageGrid(STATION_X[station], xsign=-1)
                dose=DepositionMap(grid.x_min,grid.x_max,grid.z_min,grid.z_max)
                spray_sample_count=[0]
                active_target=[0]
                def deposit(t,p,angle,stand,tube,quality):
                    if quality:
                        count=grid.stamp(p[0],p[2]);dose.deposit(p[0],p[2],stand,angle,DT_S);spray_sample_count[0]+=1
                        paint_visual.spray_sample(t,station+1,active_target[0],p[0],p[2],quality)
                    else:count=0
                    rw.writerow([round(t,3),station+1,active_target[0],
                                 *[round(v,5) for v in p],round(angle,3),round(stand,5),
                                 round(tube,5),int(quality),count])
                    if quality and spray_sample_count[0]%30==0:rf.flush()
                raster_sampling[0]=deposit
                previous_accepted=None
                try:
                    home()
                    sx=STATION_X[station]
                    for j,(dx,z) in enumerate(LOCAL_XZ,1):
                        target=(sx-dx,1.20,z);index=station*WAYPOINTS_PER_STATION+j
                        active_target[0]=index
                        # Simulated spray is never enabled on initial approach.
                        # Starting with waypoint two, its allowed geometric tube
                        # is the previous accepted pose -> current raster pose.
                        active_sweep[0]=None
                        status='NOT REACHED';err=float('inf');ang=float('inf');stand=float('nan')
                        for iteration in range(1,MAX_UPDATES+1):
                            p,a,q=read();gap=check_tip(p)
                            err=distance(target,p);ang=aim_angle_deg(a);stand=p[1]-VEHICLE_SIDE
                            wr.writerow([round(bot.getTime(),3),station+1,index,iteration,MODE,
                                         *[round(v,5) for v in target],*[round(v,5) for v in p],
                                         round(err,5),round(ang,3),*[round(v,5) for v in a],
                                         round(stand,5),round(gap,5),*[round(v,5) for v in q]])
                            accepted=(err<=XYZ_TOL and abs(stand-STANDOFF)<=XYZ_TOL and
                                      (MODE=='AUDIT' or ang<=ANGLE_TOL_DEG))
                            if accepted:
                                status='XYZ_ONLY' if MODE=='AUDIT' else 'POSE_PASS'
                                passed+=1
                                # Paint a virtual point footprint at the MEASURED accepted pose.
                                # This is not continuous liquid paint coverage.
                                # Even an accepted pose receives virtual-spray credit
                                # only if its independent spray-quality gates are healthy.
                                endpoint_credit=(linkmon.status=='PROXY_GUARD_ACTIVE' and
                                                 linkmon.guard_state=='OK' and
                                                 abs(stand-STANDOFF)<=0.035 and
                                                 ang<=ANGLE_TOL_DEG and gap>=MIN_TIP_GAP)
                                accepted_new_cells=grid.stamp(p[0],p[2]) if endpoint_credit else 0
                                if endpoint_credit: dose.deposit(p[0],p[2],stand,ang,DT_S)
                                spray_sample_count[0]+=int(endpoint_credit)
                                rw.writerow([round(bot.getTime(),3),station+1,index,
                                             *[round(v,5) for v in p],round(ang,3),round(stand,5),
                                             0.0,int(endpoint_credit),accepted_new_cells])
                                previous_accepted=tuple(p)
                                active_sweep[0]=None
                                # Color the measured, qualified surface location only.
                                # Fail-closed: an IK-only PASS is not spray credit.
                                if endpoint_credit:
                                    paint_visual.spray_waypoint(bot.getTime(),station+1,index,p[0],p[2],True)
                                print(f'[Stage 21 far arm] {status} waypoint {index}: XYZ={err:.3f}m axis_error={ang:.1f}deg standoff={stand:.3f}m')
                                break
                            if iteration==1 or (iteration-1)%RECALIBRATE_EVERY==0:J=jacobian()
                            p,a,q=read();check_tip(p)
                            current=feature(p,a,MODE)
                            goal=feature(target,AIM,MODE)
                            dq=dls(J,[v-u for v,u in zip(goal,current)])
                            command([qj+dj for qj,dj in zip(q,dq)])
                            if previous_accepted is not None:
                                active_sweep[0]=(previous_accepted,target)
                            tick(HOLD_STEPS)
                            active_sweep[0]=None
                        active_sweep[0]=None
                        sw.writerow([station+1,index,MODE,status,round(err,5),round(ang,3),round(stand,5),*[round(v,4) for v in target]])
                        fp.flush();sf.flush()
                        if status=='NOT REACHED':
                            raise RuntimeError(f'NO POSE SOLUTION waypoint {index} XYZ={err:.3f}m AIM={ang:.1f}deg')
                    active_sweep[0]=None
                    home();completed.add(station)
                    cw.writerow([station+1,WAYPOINTS_PER_STATION,passed,spray_sample_count[0],
                                 grid.events,len(grid.covered),grid.total,round(grid.percent,2),
                                 'GPS_quality_gated_disc_footprints'])
                    ds=dose.summary()
                    dpw.writerow([station+1,passed,ds['samples'],ds['patch_cells'],
                                  *[round(ds[k],6) if ds[k] is not None else 'NA' for k in (
                                  'dose_mean_index','dose_std_index','dose_cv','dose_p10_index',
                                  'dose_p90_index','dose_min_index','dose_max_index',
                                  'relative_20pct_coverage_percent','overspray_apron_fraction_percent',
                                  'inside_dose_sum','apron_dose_sum')]])
                    for record in dose.rows(station+1):
                        dgw.writerow([*record[:-1],round(record[-1],8)])
                    dpf.flush();dgf.flush()
                    print('[Stage 21 far dose] PATCH %d: modeled dose CV=%s, apron_proxy=%s, valid_samples=%d'%(
                        station+1,
                        'NA' if ds['dose_cv'] is None else '%.3f'%ds['dose_cv'],
                        'NA' if ds['overspray_apron_fraction_percent'] is None else '%.1f%%'%ds['overspray_apron_fraction_percent'],
                        ds['samples']))
                    cf.flush();rf.flush()
                    print('[Stage 22 far paint] Visible coating events=%d file=%s'%(paint_visual.events,paint_visual.log_path))
                    print(f'[Stage 21 far raster] PATCH {station+1}: poses={passed}/{WAYPOINTS_PER_STATION} '
                          f'coverage={grid.percent:.1f}% cells={len(grid.covered)}/{grid.total} '
                          f'virtual_spray_samples={spray_sample_count[0]}')
                    # Quick world can finish after one station; full world after three.
                    if (len(completed)==3 or ('--quick' in sys.argv and len(completed)==1)) and not monitor_finished[0]:
                        linkmon.summary();monitor_finished[0]=True
                    send(f'DONE:{station}:{passed}');tick(2)
                    print(f'[Stage 21 far arm] STATION {station+1} COMPLETE {passed}/{WAYPOINTS_PER_STATION}; MODE={MODE}.')
                except (RuntimeError,ValueError) as exc:
                    active_sweep[0]=None; raster_sampling[0]=None
                    print('[Stage 21 far arm] SAFETY/REACHABILITY STOP',exc)
                    send('FAIL:%d:%s'%(station,str(exc).replace(':','_')))
                    guard_emergency('STATION_FAILURE_'+str(exc))

if __name__=='__main__':main()
