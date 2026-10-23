"""Stage23 deterministic surface planner: front hood, roof, boot lid, rear tailgate.

Uses measured gantry nozzle tip for authorization, car surface geometry for
purely *visual* boxes. Body is a simplified static box; not scanned CAD mesh.
"""
from dataclasses import dataclass
import math

ORIGIN=(-2.95,-0.70,2.50)
NOZZLE_LENGTH=0.12
STANDOFF=0.25
BLUE='0.035 0.320 0.880'
@dataclass(frozen=True)
class Panel:
    name:str
    surface_height:float
    points:tuple  # row, column, (x,y,z)


def mesh(xs,ys,z):
    out=[]
    for iy,y in enumerate(ys):
        vals=xs if iy%2==0 else tuple(reversed(xs))
        for ix,x in enumerate(vals):out.append((iy+1,ix+1,(float(x),float(y),z)))
    return tuple(out)


def panels(quick=False):
    if quick:
        roof=mesh((-1.05,-0.25,0.55),(-0.53,0,0.53),1.60)
        hood=mesh((1.20,1.75,2.22),(-0.53,0,0.53),1.13)
        boot=mesh((-2.21,-1.68),(-0.53,0,0.53),1.13)
        tail=tuple((i+1,j+1,(-2.40,y,z)) for i,z in enumerate((0.72,0.98)) for j,y in enumerate((-.53,0,.53)))
    else:
        roof=mesh((-.995,-.74,-.485,-.23,.025,.28,.535,.79),(-.66,-.396,-.132,.132,.396,.66),1.60)
        hood=mesh((1.095,1.325,1.555,1.785,2.015,2.245),(-.66,-.396,-.132,.132,.396,.66),1.13)
        boot=mesh((-2.22,-2.02,-1.82,-1.62),(-.66,-.396,-.132,.132,.396,.66),1.13)
        tail=tuple((i+1,j+1,(-2.40,y,z)) for i,z in enumerate((0.64,0.78,0.92,1.06))
                   for j,y in enumerate((-.64,-.32,0,.32,.64)))
    return (Panel('roof',1.60,roof),Panel('hood',1.13,hood),Panel('boot',1.13,boot),Panel('tailgate',0.,tail))


def panel_point(panel,point):
    x,y,z=point
    if panel=='tailgate':return (-2.65,y,z)
    return (x,y,z+STANDOFF)


def expected_motor_targets(panel,point):
    x,y,z=point
    if panel=='tailgate':
        # 90 degree nozzle rotation swings its 0.12 m tip toward +X.
        # Tip target rear = -2.65, pivot = -2.77.
        pivot_x=-2.65-NOZZLE_LENGTH
        return (pivot_x-ORIGIN[0],y-ORIGIN[1],ORIGIN[2]-z,-math.pi/2)
    return (x-ORIGIN[0],y-ORIGIN[1],ORIGIN[2]-NOZZLE_LENGTH-(z+STANDOFF),0.)


def make_paint_shape(panel,point,event_id,measured=None):
    if not event_id.startswith('S23_'):raise ValueError('Bad paint name')
    x,y,z=point
    if measured is None:raise ValueError('Measured GPS is required')
    target=panel_point(panel,point)
    if math.dist(measured,target)>0.040:raise ValueError('Measured tip off paint target')
    if panel=='roof':
        width=(0.268,0.275,0.007)
        position=(x,y,1.605)
    elif panel=='hood':
        width=(0.236,0.275,0.007)
        position=(x,y,1.135)
    elif panel=='boot':
        width=(0.208,0.275,0.007)
        position=(x,y,1.135)
    elif panel=='tailgate':
        width=(0.007,0.328 if abs(y)>.1 else 0.335,0.149 if z<1.0 else 0.13)
        position=(-2.405,y,z)
    else:raise ValueError('Unknown panel')
    if not all(math.isfinite(v) for v in position+width):raise ValueError('Nonfinite paint geometry')
    tr=' '.join(f'{v:.5f}' for v in position)
    dim=' '.join(f'{v:.5f}' for v in width)
    return (f'DEF {event_id} Transform {{ translation {tr} children [ Shape {{ '
            f'appearance PBRAppearance {{ baseColor {BLUE} metalness 0.45 roughness 0.19 }} '
            f'geometry Box {{ size {dim} }} }} ] }}')
