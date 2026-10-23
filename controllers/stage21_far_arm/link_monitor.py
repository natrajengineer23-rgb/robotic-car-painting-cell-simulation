"""Stage 15: conservative link-centerline-to-car-AABB PROXY GUARD.

The monitor obtains link origins from Webots Supervisor device -> hinge joint ->
endPoint Solid. These point origins are only proxies for link volumes. It logs
clearance samples but cannot certify collision freedom. Measurement failures
are fail-closed in the arm controller; link centerlines are NOT collision meshes.
"""
import csv, math
from pathlib import Path

CAR_BOXES=(
    ((-2.40,-.95,.51),(2.40,.95,1.13)),
    ((-1.415,-.875,1.15),(.935,.875,1.60)),
)
LINK_RADIUS_M=.075  # assumed centerline radius ONLY for approximate screening
NOZZLE_RADIUS_M=.025
SPACING_M=.025
WARN_PROXY_M=.140
STOP_PROXY_M=.100

def classify_proxy(clearance_m):
    if not math.isfinite(clearance_m): return "INVALID"
    if clearance_m < STOP_PROXY_M: return "STOP"
    if clearance_m < WARN_PROXY_M: return "WARNING"
    return "OK"

def point_box_gap(p,lo,hi):
    return math.sqrt(sum(max(lo[i]-p[i],0.0,p[i]-hi[i])**2 for i in range(3)))

def segment_box_gap(a,b,lo,hi,spacing=SPACING_M):
    d=math.dist(a,b)
    n=max(1, math.ceil(d/spacing))
    return min(point_box_gap([a[k]+(b[k]-a[k])*t/n for k in range(3)],lo,hi)
               for t in range(n+1))

class LinkMonitor:
    def __init__(self,supervisor,motors,tip_gps,base_gps,output_file,robot_def):
        self.bot=supervisor;self.tip=tip_gps;self.base=base_gps
        self.arm=supervisor.getFromDef(robot_def)
        self.links=[];self.missing=[]
        for motor in motors:
            name=motor.getName()
            try:
                # Webots R2025a's Supervisor.getFromDevice expects an integer
                # WbDeviceTag, not the Python RotationalMotor wrapper object.
                # controller.device.Device stores that tag as `_tag`.
                tag=getattr(motor, '_tag', None)
                if tag is None:
                    get_tag=getattr(motor,'getTag',None)
                    tag=get_tag() if callable(get_tag) else None
                if not isinstance(tag,int) or tag<=0:
                    raise ValueError(f'Invalid Webots device tag for {name}: {tag!r}')
                dev=supervisor.getFromDevice(tag)
                if dev is None:
                    raise ValueError(f'No Supervisor node for motor tag {tag}')
                joint=dev.getParentNode()
                if joint is None or joint.getBaseTypeName()!='HingeJoint':
                    raise ValueError(f'Expected parent HingeJoint for {name}')
                ef=joint.getField('endPoint') if joint is not None else None
                node=ef.getSFNode() if ef is not None else None
                if node is None or node.getBaseTypeName() not in ('Solid','Robot'):
                    raise ValueError('motor endPoint Solid not accessible')
                p=node.getPosition()
                if p is None or len(p)!=3:
                    raise ValueError('link world position not readable')
                self.links.append((name,node))
            except Exception as exc:
                self.missing.append(f'{name}: {type(exc).__name__}: {str(exc)[:80]}')
        self.path=Path(output_file)
        self.path.parent.mkdir(parents=True,exist_ok=True)
        self.fp=self.path.open('w',newline='')
        self.writer=csv.writer(self.fp)
        self.writer.writerow(['time_s','status','segment','centerline_gap_m','proxy_surface_clearance_m','proxy_radius_m','links_resolved','links_total','guard_state','warn_threshold_m','stop_threshold_m'])
        self.sample_count=0
        self.last_clearance=None;self.last_segment='N/A';self.guard_state='UNVERIFIED'
        self.warning_count=0;self.stop_count=0
        self.near_count=0
        self.closest=float('inf');self.closest_segment=''
        if len(self.links)==len(motors) and self.arm is not None:
            self.status='PROXY_GUARD_ACTIVE'
        elif len(self.links)>=2:
            self.status='PARTIAL_MONITOR'
        else:
            self.status='MONITOR_UNAVAILABLE'
        print(f'[Stage 17 link] {self.status}: accessible joint endpoints {len(self.links)}/{len(motors)}')
        for missing in self.missing:print('[Stage 21 far link] UNAVAILABLE:',missing)
        if self.arm is None:print('[Stage 21 far link] UNAVAILABLE: robot base DEF PAINT_ROBOT')
        if self.status=='MONITOR_UNAVAILABLE':
            print('[Stage 21 far link] NOT TESTED: no valid link clearance measurements; collision clearance UNKNOWN.')
        print('[Stage 21 far link] Conservative centerline proxies, NOT full mesh collision clearance.')

    def observe(self):
        if self.status!='PROXY_GUARD_ACTIVE':return
        self.sample_count+=1
        try:
            positions=[]
            if self.arm is not None:positions.append(('arm_base',list(self.arm.getPosition())))
            positions.extend((name,list(node.getPosition())) for name,node in self.links)
            segments=[]
            for i in range(1,len(positions)):
                segments.append((positions[i-1][0]+'__'+positions[i][0],positions[i-1][1],positions[i][1],LINK_RADIUS_M))
            tip=list(self.tip.getValues());base=list(self.base.getValues())
            segments.append(('nozzle_axis',base,tip,NOZZLE_RADIUS_M))
            values=[]
            for name,a,b,radius in segments:
                raw=min(segment_box_gap(a,b,lo,hi) for lo,hi in CAR_BOXES)
                val=raw-radius
                values.append((val,name,raw,radius))
            if not values:raise ValueError('No valid proxy segments')
            val,name,raw,radius=min(values)
            self.last_clearance=val;self.last_segment=name
            self.guard_state=classify_proxy(val)
            if val<self.closest:self.closest=val;self.closest_segment=name
            if val<0:self.near_count+=1
            if self.guard_state=='WARNING':self.warning_count+=1
            if self.guard_state in ('STOP','INVALID'):self.stop_count+=1
            if self.sample_count%5==0 or self.guard_state!='OK':
                self.writer.writerow([round(self.bot.getTime(),4),self.status,name,
                                     round(raw,5),round(val,5),radius,len(self.links),6,
                                     self.guard_state,WARN_PROXY_M,STOP_PROXY_M])
                self.fp.flush()
        except Exception as exc:
            self.status='MONITOR_UNAVAILABLE'
            self.guard_state='INVALID'
            self.last_clearance=None
            print('[Stage 21 far link] MONITOR_UNAVAILABLE while sampling:',type(exc).__name__,str(exc))

    def summary(self):
        minimum=(f'{self.closest:.4f}m' if math.isfinite(self.closest) else 'N/A (no measurements)')
        print(f'[Stage 17 link] GUARD SUMMARY: status={self.status}, sampled_steps={self.sample_count}, '+
              f'proxy_negative_clearance_samples={self.near_count}, '+
              f'min_proxy_clearance={minimum} segment={self.closest_segment or "N/A"}')
        print(f'[Stage 17 link] TRACE: {self.path}')
        print('[Stage 21 far link] NOTE: negative proxy clearance is a screening flag. '+
              'Nonnegative values DO NOT prove collision-free full links.')
        if not self.fp.closed:
            self.fp.flush();self.fp.close()
