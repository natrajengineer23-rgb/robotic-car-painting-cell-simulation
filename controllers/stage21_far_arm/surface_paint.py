"""Stage 22 visible virtual coating on the car-side panels.

A visual deposition layer is created only from quality-gated measured tool
poses. It is NOT a fluid simulation, calibrated film thickness, or a claim
of whole-vehicle coverage. Shapes have no boundingObject or Physics.

Maintains the upstream Stage 21 interlocks; this module NEVER commands motion.
"""
import csv
import math
from pathlib import Path

# Existing Stage 21 scene side panel spans y=+/-0.95, z approx 0.51..1.13.
SIDE_FACE_M = 0.950
COAT_CENTRE_M = 0.955
# Indigo-blue glossy layer contrasts with the baseline red painted body.
COAT_RGB = (0.035, 0.32, 0.88)
# A small coverage tile centered on each validated pose, augmented by
# periodic quality-gated samples along the trajectory.
WAYPOINT_TILE_M = (0.085, 0.006, 0.105)
SAMPLE_RADIUS_M = 0.051
SAMPLE_EVERY = 24


def valid_surface_point(x, z):
    return (math.isfinite(x) and math.isfinite(z)
            and -2.35 <= x <= 2.35 and 0.57 <= z <= 1.07)


def paint_shape(x, z, side, kind, event_id):
    """VRML Transform for non-colliding *visual* paint on an exterior panel."""
    if side not in ('near', 'far'):
        raise ValueError('Unknown side')
    if kind not in ('waypoint', 'sample'):
        raise ValueError('Unknown paint event')
    if not valid_surface_point(x, z):
        raise ValueError('Paint point off side panel')
    y = -COAT_CENTRE_M if side == 'near' else COAT_CENTRE_M
    rgb = ' '.join(f'{v:.3f}' for v in COAT_RGB)
    if kind == 'waypoint':
        a,b,c = WAYPOINT_TILE_M
        shape=f'Box {{ size {a:.4f} {b:.4f} {c:.4f} }}'
    else:
        # A Webots Cylinder has its axis along Y, so the disc lies on x/z.
        shape=(f'Cylinder {{ radius {SAMPLE_RADIUS_M:.4f} '
               'height 0.0040 subdivision 16 }')
    return (f'DEF PAINT_{side.upper()}_{event_id} Transform {{ '
            f'translation {x:.5f} {y:.5f} {z:.5f} '
            'children [ Shape { '
            f'appearance PBRAppearance {{ baseColor {rgb} metalness 0.55 roughness 0.22 }} '
            f'geometry {shape} '
            '} ] }')


class SurfacePainter:
    """Track and draw simulated paint, never independently authorize spraying."""
    def __init__(self, root_children, log_dir, side):
        if side not in ('near', 'far'):
            raise ValueError('side must be near/far')
        self.root_children=root_children
        self.side=side
        self.events=0
        self.samples_seen=0
        self.errors=0
        self.log_path=Path(log_dir)/f'stage22_{side}_paint_events.csv'
        self.log_path.parent.mkdir(parents=True,exist_ok=True)
        self.file=self.log_path.open('w',newline='')
        self.writer=csv.writer(self.file)
        self.writer.writerow(('time_s','station','target','kind','x_m','y_surface_m','z_m','mode','painted'))
        self.file.flush()

    def _add(self, time_s, station, target, x, z, kind, quality):
        # Every event is gated against the caller's pose/safety qualification.
        if not quality or not valid_surface_point(x,z):
            return False
        event_id=self.events+1
        try:
            node=paint_shape(x,z,self.side,kind,event_id)
            self.root_children.importMFNodeFromString(-1,node)
        except Exception as error:
            self.errors+=1
            if self.errors<=3:
                print(f'[Stage 22 {self.side} paint] VISUALIZATION WARNING: {error}')
            return False
        self.events+=1
        y=-COAT_CENTRE_M if self.side=='near' else COAT_CENTRE_M
        self.writer.writerow((round(time_s,3),station,target,kind,round(x,5),y,round(z,5),'VIRTUAL',1))
        self.file.flush()
        return True

    def spray_sample(self, time_s, station, target, x, z, quality):
        if not quality:
            return False
        self.samples_seen+=1
        if self.samples_seen % SAMPLE_EVERY:
            return False
        return self._add(time_s,station,target,x,z,'sample',quality)

    def spray_waypoint(self, time_s, station, target, x, z, quality):
        return self._add(time_s,station,target,x,z,'waypoint',quality)

    def close(self):
        if not self.file.closed:self.file.close()
