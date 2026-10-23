"""Stage 23: powered three-axis overhead virtual car-painting module.

Two existing UR5e rail paths continue independently. This gantry is a new
physical XYZ-slider + servo-tilt actuator, NOT a third UR5e. It covers roof,
hood, boot lid and rear tailgate using *measured GPS* and encoder checks.
Coating is solely a non-colliding Webots visual overlay, not fluid physics.
No complete robot-link/gantry collision model or safety-rated protection.
"""
from controller import Supervisor
from pathlib import Path
import math, csv, sys
from surface_planner import panels, panel_point, make_paint_shape, expected_motor_targets

STEP_MS=32
ROOT=(-2.95,-0.70,2.50)
POSITION_TOL=0.040
ENCODER_TOL=0.020
ROTATION_TOL=0.06
RETRACT_QZ=0.10
MAX_STEPS=480  # 15.36 s per move; allows bounded acceleration for long X travel
# Joint stop margins are monitored without assuming physics collision proof.
MECHANICAL_BOUNDS=((-0.05,5.95),(-0.05,1.45),(-0.05,1.95),(-1.65,0.11))
SOFT_BOUNDS=((0.0,5.90),(0.0,1.40),(0.0,1.92),(-1.60,0.08))
COATING_COLOR=(0.035,0.32,0.88)

def distance(a,b):
    return math.sqrt(sum((float(x)-float(y))**2 for x,y in zip(a,b)))

def main():
    bot=Supervisor()
    quick='--quick' in sys.argv
    x=bot.getDevice('gantry_x_motor')
    y=bot.getDevice('gantry_y_motor')
    z=bot.getDevice('gantry_z_motor')
    tilt=bot.getDevice('nozzle_tilt_motor')
    gps=bot.getDevice('stage23_nozzle_tip_gps')
    rx=bot.getDevice('cell_stop_in')
    tx=bot.getDevice('cell_stop_out')
    if any(dev is None for dev in (x,y,z,tilt,gps,rx,tx)):
        raise RuntimeError('Stage23: missing gantry actuator, GPS, or interlock receiver')
    motors=(x,y,z,tilt)
    sensors=[]
    for motor in motors:
        enc=motor.getPositionSensor()
        if enc is None:raise RuntimeError('Stage23 missing encoder')
        enc.enable(STEP_MS)
        sensors.append(enc)
    gps.enable(STEP_MS)
    rx.enable(STEP_MS)
    children=bot.getRoot().getField('children')
    folder=Path(__file__).resolve().parents[2]/'logs'/'overhead'
    folder.mkdir(parents=True,exist_ok=True)
    out=folder/('stage23_overhead_quick.csv' if quick else 'stage23_overhead_full.csv')
    print('[Stage 23] MODE:', 'QUICK' if quick else 'FULL')
    print('[Stage 23] Third painting mechanism: motorized XYZ overhead gantry + tilt; NOT a UR5e.')
    print('[Stage 23] BLUE virtual coating only, quality-gated from actual GPS and motor encoders.')
    print('[Stage 23] Simulated geometry with proxy approach checks; NOT certified collision safety.')
    def step():
        if bot.step(STEP_MS)==-1:raise SystemExit
        while rx.getQueueLength():
            getstring=getattr(rx,'getString',None)
            raw=getstring() if callable(getstring) else rx.getData()
            msg=raw.decode('utf8','replace') if isinstance(raw,(bytes,bytearray)) else str(raw)
            rx.nextPacket()
            if 'STOP:' in msg or 'FAIL:' in msg:
                raise RuntimeError('Cell-level stop received: '+msg)
    def measured():return tuple(s.getValue() for s in sensors)
    def validate_encoder(e,label):
        if any(not math.isfinite(v) for v in e):
            raise RuntimeError('GANTRY_ENCODER_INVALID '+label+' enc='+str(e))
        for axis,(value,(lo,hi)) in enumerate(zip(e,MECHANICAL_BOUNDS)):
            if value<lo-0.025 or value>hi+0.025:
                raise RuntimeError('GANTRY_ENCODER_OUT_OF_BOUNDS '+label+
                                   ' axis='+str(axis)+' value='+str(value)+' bounds='+str((lo,hi)))
    def command(q):
        for axis,(motor,v,(lo,hi)) in enumerate(zip(motors,q,SOFT_BOUNDS)):
            if not math.isfinite(v) or v<lo-1e-6 or v>hi+1e-6:
                raise ValueError('GANTRY_COMMAND_OUT_OF_BOUNDS axis='+str(axis)+' value='+str(v))
            motor.setPosition(min(hi,max(lo,v)))

    def settle(q,label):
        command(q)
        stable=0
        for i in range(MAX_STEPS):
            step()
            e=measured()
            validate_encoder(e,label)
            if max(abs(v-t) for v,t in zip(q,e))<ENCODER_TOL:
                stable+=1
                if stable>=8:return
            else: stable=0
        raise RuntimeError('GANTRY_MOTION_TIMEOUT '+label+' enc='+str(measured())+' target='+str(q))
    results=[]
    with out.open('w',newline='') as fp:
        wr=csv.writer(fp)
        wr.writerow(('panel','row','column','target_x','target_y','target_z','tip_x','tip_y','tip_z',
                     'position_error_m','encoder_error','tilt_error_rad','surface_gap_m','result','paint_id'))
        event=0
        try:
            # No painting is allowed if startup state is outside limits.
            step()
            validate_encoder(measured(),'initial')
            settle((0,0,RETRACT_QZ,0),'home')
            print('[Stage 23] GANTRY_HOME_VERIFIED enc=',measured(),'tipGPS=',gps.getValues())
            for panel in panels(quick):
                count=0
                for row,col,point in panel.points:
                    target_xyz=panel_point(panel.name,point)
                    q=expected_motor_targets(panel.name,point)
                    # Retract and level the nozzle *before* moving laterally.
                    prev=measured()
                    settle((prev[0],prev[1],RETRACT_QZ,0),'clearance_retract')
                    settle((q[0],q[1],RETRACT_QZ,0),'lateral_travel')
                    # Tilt head at safe height before lowering to the target.
                    settle((q[0],q[1],RETRACT_QZ,q[3]),'safe_tilt')
                    settle(q,'final_approach')
                    p=tuple(float(v) for v in gps.getValues())
                    enc=measured()
                    err=distance(p,target_xyz)
                    encerr=max(abs(a-b) for a,b in zip(enc,q))
                    tilt_err=abs(enc[3]-q[3])
                    # Only the intended surface and side of the panel are accepted.
                    surf_gap=(-2.40-p[0]) if panel.name=='tailgate' else (p[2]-panel.surface_height)
                    gate=(err<=POSITION_TOL and encerr<=ENCODER_TOL and tilt_err<=ROTATION_TOL and
                          abs(surf_gap-0.25)<=0.04 and all(math.isfinite(v) for v in p))
                    status='POSE_PASS' if gate else 'NO_COAT'
                    paint_id=''
                    if gate:
                        event+=1
                        paint_id=f'S23_{panel.name.upper()}_{event}'
                        # Paint deposited on the actual model face, never at an unverified location.
                        geom=make_paint_shape(panel.name,point,paint_id,p)
                        children.importMFNodeFromString(-1,geom)
                        count+=1
                    wr.writerow((panel.name,row,col,*[round(v,5) for v in target_xyz],
                        *[round(v,5) for v in p],round(err,5),round(encerr,5),
                        round(tilt_err,5),round(surf_gap,5),status,paint_id))
                    fp.flush()
                    if not gate:
                        print('[Stage 23] SKIPPED',panel.name,'row',row,'col',col,'error',round(err,3))
                    elif count%5==0:
                        print('[Stage 23] COATED',panel.name,'row',row,'col',col)
                results.append((panel.name,count,len(panel.points)))
                print('[Stage 23] PANEL',panel.name,': qualified',count,'/',len(panel.points))
            settle((measured()[0],measured()[1],RETRACT_QZ,0),'final_retract')
            settle((0,0,RETRACT_QZ,0),'return_home')
            print('[Stage 23] FINISHED:',' '.join('%s=%d/%d'%p for p in results))
            print('[Stage 23] LOG:',out)
        except (RuntimeError,ValueError) as exc:
            reason=str(exc)
            print('[Stage 23] STOP/FAILED:',reason)
            # Existing near/far rail controllers consume CELL_STOP broadcasts on ch 33.
            # Never declare success after a gantry failure.
            try:
                tx.send(('CELL_STOP:overhead:'+reason.replace(':','_')).encode('utf8'))
            except Exception as send_error:
                print('[Stage 23] CELL_STOP_SEND_ERROR:',send_error)
            try:
                e=measured()
                for motor,value,(lo,hi) in zip(motors,e,SOFT_BOUNDS):
                    if math.isfinite(value):motor.setPosition(min(hi,max(lo,value)))
                bot.step(STEP_MS)  # flush interlock radio to the other rails
            except Exception:pass
            raise

if __name__=='__main__':main()
