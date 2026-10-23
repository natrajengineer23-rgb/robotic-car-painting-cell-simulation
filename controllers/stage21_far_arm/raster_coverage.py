"""Stage 17 isolated local-patch serpentine raster and virtual spray-coverage grid.

This is a 2D simulation of deposited *footprints* driven by measured nozzle GPS
while pose quality gates are satisfied. It is NOT paint fluid, a continuous-path
verification, an industrial coverage metric, or full-car coverage.
"""
import math

X_OFFSETS = (0.30, 0.35, 0.40, 0.45, 0.50)
Z_ROWS = (0.68, 0.765, 0.85)
WAYPOINTS_PER_STATION = len(X_OFFSETS) * len(Z_ROWS)
GRID_M = 0.010
FOOTPRINT_RADIUS_M = 0.060
PATH_TUBE_M = 0.045


def make_raster():
    seq=[]
    for row,z in enumerate(Z_ROWS):
        xs = X_OFFSETS if row%2==0 else tuple(reversed(X_OFFSETS))
        for dx in xs: seq.append((dx,z,row))
    return tuple(seq)

RASTER=make_raster()


def point_segment_gap_xz(px,pz,ax,az,bx,bz):
    vx,vz=bx-ax,bz-az
    vv=vx*vx+vz*vz
    t=max(0.,min(1.,((px-ax)*vx+(pz-az)*vz)/vv)) if vv>1e-12 else 0.
    return math.hypot(px-(ax+t*vx),pz-(az+t*vz))


class CoverageGrid:
    """Boolean x-z raster for a *single* 0.20 m by 0.17 m local car patch.

    For each sample, rasterize a circular virtual footprint centered at actual
    observed spray-tip projection. This ignores fluid dynamics, paint rate and
    angular footprints. An accepted sample means the external quality gates
    passed; this class does not decide safety.
    """
    def __init__(self,station_x,grid=GRID_M,footprint=FOOTPRINT_RADIUS_M,xsign=-1):
        self.station_x=station_x
        self.x_min=min(station_x+xsign*t for t in X_OFFSETS)
        self.x_max=max(station_x+xsign*t for t in X_OFFSETS)
        self.z_min=min(Z_ROWS);self.z_max=max(Z_ROWS)
        self.grid=grid;self.footprint=footprint
        self.nx=round((self.x_max-self.x_min)/grid)
        self.nz=round((self.z_max-self.z_min)/grid)
        self.covered=set()
        self.samples=0
        self.events=0
    @property
    def total(self):return self.nx*self.nz
    @property
    def percent(self):return 100.*len(self.covered)/self.total
    def stamp(self,x,z):
        self.samples+=1
        new=0
        for iz in range(self.nz):
            cz=self.z_min+(iz+.5)*self.grid
            if abs(cz-z)>self.footprint:continue
            for ix in range(self.nx):
                cx=self.x_min+(ix+.5)*self.grid
                if (cx-x)**2+(cz-z)**2 <= self.footprint**2:
                    cell=(ix,iz)
                    if cell not in self.covered:
                        self.covered.add(cell);new+=1
        self.events+=int(new>0)
        return new
