"""Stage 17: powered SliderJoint and UR5e painting-station handshake.

Station handshake protocol (UTF-8):
  rail -> arm: STATION:<index>
  arm -> rail: STARTED:<index>, DONE:<index>:<passed>, or FAIL:<index>:<reason>
No Supervisor carriage repositioning is used. The rail only travels while the
arm is homed. Fixed far-side rail, NOT an end-around/far-side transfer.
"""
from controller import Robot
from pathlib import Path
import csv
import sys
RASTER_POINTS_PER_STATION=15

STEP=32
ROOT_X=2.50
ROOT_Y=1.55
ROOT_Z=0.270
X_SIGN=-1
STATION_OFFSETS=(0.40,1.95,3.50)
STATION_X=(2.10,0.55,-1.00)
ENC_TOL=0.040
GPS_X_TOL=0.080
GPS_Y_TOL=0.12
GPS_Z_TOL=0.13
RAIL_SPEED=0.28
MAX_MOVE_S=32.0
MAX_PAINT_S=1100.0
MAX_ARM_READY_S=35.0
STABLE_POS_TOL=0.018   # 18 mm, tighter than the 65 mm painting guard
STABLE_VEL_MPS=0.025 # rail must actually stop before painting starts
STABLE_SAMPLES=32     # 1.024 s at STEP=32 ms
DIAGNOSTIC_EVERY=16   # telemetry samples (0.512 s)


def read_radio_text(receiver):
    """Read one Webots radio message on R2025a or older Python bindings."""
    get_string = getattr(receiver, 'getString', None)
    data = get_string() if callable(get_string) else receiver.getData()
    if isinstance(data, (bytes, bytearray)):
        data = data.decode('utf-8', 'replace')
    return data.strip('\x00')


def main():
    bot=Robot()
    SIDE='far'
    full='--full' in sys.argv
    inject_cell_stop='--inject-cell-stop' in sys.argv
    if inject_cell_stop and full:
        raise RuntimeError('Fault injection world must use --quick only')
    stations=list(range(3)) if full else [0]
    motor=bot.getDevice('rail_motor')
    encoder=bot.getDevice('rail_encoder')
    gps=bot.getDevice('rail_carriage_gps')
    radio_out=bot.getDevice('rail_to_arm')
    radio_in=bot.getDevice('arm_to_rail')
    cell_tx=bot.getDevice('cell_stop_out')
    cell_rx=bot.getDevice('cell_stop_in')
    if any(v is None for v in (motor,encoder,gps,radio_out,radio_in,cell_tx,cell_rx)):
        print('[Stage 21 far rail] FATAL missing rail motor, sensor or radio device')
        return
    encoder.enable(STEP)
    gps.enable(STEP)
    radio_in.enable(STEP)
    cell_rx.enable(STEP)
    motor.setVelocity(RAIL_SPEED)
    motor.setPosition(STATION_OFFSETS[0])
    root=Path(__file__).resolve().parents[2]
    logs=root/'logs'/'far';logs.mkdir(parents=True,exist_ok=True)
    output=logs/'stage21_far_rail_stations.csv'
    diag_path=logs/'stage21_far_rail_diagnostics.csv'
    print('[Stage 21 far rail] MODE:', 'FULL (3 stations, 45 far-side raster targets)' if full else 'QUICK (1 station, 15 far-side raster targets)')
    print('[Stage 21 far rail] Powered SliderJoint, no Supervisor carriage teleportation.')
    if inject_cell_stop:print('[Stage 21 far TEST] ARMED fault on far rail; will broadcast shared STOP 4.0 simulated seconds after ARM STARTED.')
    print('[Stage 21 far rail] End-stop clearance: home q=0.40m, last station q=3.50m, maxStop=3.90m.')
    print('[Stage 21 far rail] Original painting-world station X values retained:', STATION_X)
    print('[Stage 21 far rail] Waiting for arm controller handshakes over Webots radio channels 11/12.')
    completed=0;failure=None

    def send(message):radio_out.send(message.encode('utf-8'))
    def broadcast_stop(reason):
        cell_tx.send(('CELL_STOP:%s:%s'%(SIDE,reason)).encode('utf-8'))
    def check_cell_stop():
        while cell_rx.getQueueLength()>0:
            message=read_radio_text(cell_rx)
            cell_rx.nextPacket()
            if message.startswith('CELL_STOP:'):
                raise RuntimeError('SHARED_CELL_INTERLOCK '+message)

    def receive():
        result=[]
        while radio_in.getQueueLength()>0:
            result.append(read_radio_text(radio_in))
            radio_in.nextPacket()
        return result
    def sample(expected=None):
        check_cell_stop()
        q=encoder.getValue()
        p=gps.getValues()
        if q < -0.08 or q > 3.98:
            raise RuntimeError('RAIL LIMIT: encoder %.3f'%q)
        if abs(p[1]-ROOT_Y)>GPS_Y_TOL or abs(p[2]-ROOT_Z)>GPS_Z_TOL:
            raise RuntimeError('RAIL DRIFT: GPS=(%.3f %.3f %.3f)'%tuple(p))
        if abs(p[0]-(ROOT_X+X_SIGN*q))>0.095:
            raise RuntimeError('ENCODER/GPS DISAGREEMENT: q=%.3f GPS X=%.3f'%(q,p[0]))
        if expected is not None:
            ok=abs(q-expected)<=ENC_TOL and abs(p[0]-(ROOT_X+X_SIGN*expected))<=GPS_X_TOL
        else:ok=True
        return q,p,ok

    with output.open('w',newline='') as fp, diag_path.open('w',newline='') as dfp:
        wr=csv.writer(fp)
        dw=csv.writer(dfp)
        wr.writerow(['station','target_rail_m','encoder_m','carriage_gps_x','carriage_gps_y','carriage_gps_z','painting_targets_passed','status','time_s'])
        dw.writerow(['time_s','phase','station','command_m','encoder_m','gps_x_m','gps_y_m','gps_z_m','rail_velocity_mps','rail_position_error_m'])
        last_q=None; last_t=None; diagnostic_count=0
        def observation(phase, station, target, force_log=False):
            nonlocal last_q, last_t, diagnostic_count
            q,p,_=sample()
            t=bot.getTime()
            v=0.0 if last_q is None or t<=last_t else (q-last_q)/(t-last_t)
            last_q=q; last_t=t
            diagnostic_count+=1
            if force_log or diagnostic_count % DIAGNOSTIC_EVERY == 0:
                dw.writerow([round(t,3),phase,station+1 if station is not None else 0,
                             round(target,5),round(q,5),*[round(z,5) for z in p],
                             round(v,5),round(q-target,5)])
                dfp.flush()
            return q,p,v

        def settled(target, phase, station, timeout_s):
            '''Keep the motor engaged until measured position AND velocity remain stable.'''
            stable=0
            start=bot.getTime()
            motor.setPosition(target)
            while bot.getTime()-start < timeout_s:
                if bot.step(STEP)==-1:raise SystemExit
                q,p,v=observation(phase,station,target)
                position_ok=(abs(q-target)<=STABLE_POS_TOL and
                             abs(p[0]-(ROOT_X+X_SIGN*target))<=STABLE_POS_TOL+0.010)
                if position_ok and abs(v)<STABLE_VEL_MPS:
                    stable+=1
                else:
                    stable=0
                if stable>=STABLE_SAMPLES:
                    observation(phase+'-PASS',station,target,True)
                    print(f'[Stage 21 far rail] SETTLED: encoder={q:.4f}m GPSx={p[0]:+.4f}m rail_velocity={v:+.4f}m/s stable_for={stable*STEP/1000:.2f}s')
                    return q,p
            observation(phase+'-TIMEOUT',station,target,True)
            raise RuntimeError(f'RAIL NOT SETTLED: phase={phase} q={q:.3f} gpsx={p[0]:.3f} v={v:.3f}')

        try:
            # Unlike Stage 8, Stage 13 commands moving arm joints during startup.
            # Do not mark first rail station ready until the manipulator reports its homing complete.
            print('[Stage 21 far rail] PRE-FLIGHT: waiting for ARM_READY after articulated arm finishes homing.')
            motor.setPosition(STATION_OFFSETS[0])
            ready=False
            ready_start=bot.getTime()
            while bot.getTime()-ready_start<MAX_ARM_READY_S:
                if bot.step(STEP)==-1:return
                observation('preflight-arm-home',None,STATION_OFFSETS[0])
                for msg in receive():
                    if msg=='ARM_READY':
                        ready=True
                    elif msg.startswith('FAIL:') or msg.startswith('STOP:'):
                        raise RuntimeError('ARM PREFLIGHT ERROR: '+msg)
                if ready:break
            if not ready:raise RuntimeError('ARM_READY handshake timeout; cannot begin rail travel')
            print('[Stage 21 far rail] ARM_READY received; testing whether the carriage is stationary under the robot load.')
            settled(STATION_OFFSETS[0],'preflight-rail-settle',None,20.0)
            print('[Stage 21 far rail] PREFLIGHT PASSED: arm homed and carriage stable. Starting station program.')
            for i in stations:
                target=STATION_OFFSETS[i]
                motor.setVelocity(RAIL_SPEED)
                motor.setPosition(target)
                print(f'[Stage 21 far rail] MOVE to station {i+1}/{len(stations)}: x={STATION_X[i]:+.3f}m, motor={target:.3f}m')
                try:
                    # Require genuinely stationary carriage before releasing the arm.
                    q,p=settled(target,'station-arrival',i,MAX_MOVE_S)
                    print(f'[Stage 21 far rail] AT STATION {i+1}: encoder={q:.3f}m GPSx={p[0]:+.3f}m, stable')
                    start=bot.getTime();ack=False;ack_time=None;last_ping=-1e10;done=None
                    while bot.getTime()-start<MAX_PAINT_S:
                        if bot.step(STEP)==-1:return
                        q,p,v=observation('station-guard',i,target)
                        if inject_cell_stop and ack_time is not None and bot.getTime()-ack_time>=4.0:
                            print('[Stage 21 far TEST] Intentional simulated fault injected during painting. Not a real collision.')
                            raise RuntimeError('INJECTED_TEST_CELL_STOP_FROM_FAR')
                        # Do not weaken the existing safety guard.
                        if abs(q-target)>0.065 or abs(p[0]-(ROOT_X+X_SIGN*target))>0.085:
                            observation('STATION-DRIFT',i,target,True)
                            raise RuntimeError('STATION DRIFT DURING PAINTING q=%.3f GPSx=%.3f v=%.3f'%(q,p[0],v))
                        if not ack and bot.getTime()-last_ping>=0.5:
                            send('STATION:%d'%i);last_ping=bot.getTime()
                        for msg in receive():
                            if msg=='STARTED:%d'%i:
                                if not ack:print('[Stage 21 far rail] ARM STARTED station',i+1)
                                ack=True
                                if ack_time is None: ack_time=bot.getTime()
                            elif msg.startswith('DONE:%d:'%i):
                                parts=msg.split(':')
                                done=int(parts[2]);break
                            elif msg.startswith('FAIL:%d:'%i):
                                raise RuntimeError('ARM FAILED: '+msg)
                            elif msg.startswith('STOP:'):
                                raise RuntimeError('ARM STOP: '+msg)
                        if done is not None:break
                    else:raise RuntimeError('ARM HANDSHAKE/PAINT TIMEOUT station %d'%(i+1))
                    if done!=RASTER_POINTS_PER_STATION:raise RuntimeError('ARM REPORTED %s/%d targets at station %d'%(done,RASTER_POINTS_PER_STATION,i+1))
                    completed+=1
                    wr.writerow([i+1,target,round(q,4),round(p[0],4),round(p[1],4),round(p[2],4),done,'PASS',round(bot.getTime(),3)])
                    fp.flush()
                    print(f'[Stage 21 far rail] STATION {i+1} PASS: 15/15 raster poses; rail motor held during arm tracking')
                except (RuntimeError, ValueError) as exc:
                    failure=str(exc)
                    print('[Stage 21 far rail] SAFETY STOP:',failure)
                    broadcast_stop(failure)
                    try:send('STOP:'+failure.replace(':','_'))
                    except Exception:pass
                    motor.setPosition(encoder.getValue())
                    wr.writerow([i+1,target,round(encoder.getValue(),4),'','','','',failure,round(bot.getTime(),3)])
                    fp.flush()
                    break
        except (RuntimeError,ValueError) as exc:
            failure=str(exc)
            print('[Stage 21 far rail] PRE-FLIGHT SAFETY STOP:',failure)
            broadcast_stop(failure)
            try:send('STOP:'+failure.replace(':','_'))
            except Exception:pass
            motor.setPosition(encoder.getValue())
    print(f'[Stage 21 far rail] FINISHED: {completed}/{len(stations)} stations; {completed*RASTER_POINTS_PER_STATION}/{len(stations)*RASTER_POINTS_PER_STATION} Cartesian raster poses confirmed; failure={failure or "none"}')
    print('[Stage 21 far rail] Far-side powered SliderJoint + encoder/GPS experiment. Independent rail; no cross-side robot transfer.')
    print('[Stage 21 far rail] CSV:',output)
    print('[Stage 21 far rail] Rail dynamics diagnostic CSV:',diag_path)
    while bot.step(STEP)!=-1:pass

if __name__=='__main__':main()
