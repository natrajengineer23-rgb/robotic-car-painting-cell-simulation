"""Stage 18: quality-gated *virtual* deposition index in a finite panel window.

NOT measured film thickness, not aerosol CFD, not evidence of paint transfer
or industrial pass/fail quality. The parametric Gaussian spray fan and
inverse-square standoff scaling are modeling assumptions to compare paths.
Only sensor-validated spray-enabled samples are added by the arm controller.
"""
import math
import statistics

PATCH_STEP=0.010       # 10 mm surface discretization
SIGMA_AT_250MM=0.040  # assumed Gaussian width (m) at nominal 250 mm standoff
NOMINAL_STANDOFF=0.250
MARGIN_M=0.160        # exterior "overspray accounting" window on all four sides
SUPPORT_SIGMAS=3.0
DT_S=0.032            # each credited sample represents one 32ms Webots step


def quantile(values, q):
    seq=sorted(values)
    if not seq: return 0.0
    pos=(len(seq)-1)*q
    low=int(math.floor(pos)); high=int(math.ceil(pos))
    return seq[low]*(high-pos)+seq[high]*(pos-low) if high!=low else seq[low]


class DepositionMap:
    """Integrated relative Gaussian dose on an x-z panel and surrounding apron.

    'Dose' is an arbitrary exposure index proportional to simulated time.
    Exterior-window dose is an *overspray potential proxy*, not aerosol waste.
    The model deliberately does not label micrometres or physical film build.
    """
    def __init__(self, x_min, x_max, z_min, z_max, step=PATCH_STEP, margin=MARGIN_M):
        if not (x_max>x_min and z_max>z_min and step>0 and margin>=0):
            raise ValueError('Invalid deposition-grid geometry')
        self.x_min,self.x_max,self.z_min,self.z_max=x_min,x_max,z_min,z_max
        self.step=step;self.margin=margin
        self.ex_min=x_min-margin;self.ez_min=z_min-margin
        self.nx=round((x_max-x_min+2*margin)/step)
        self.nz=round((z_max-z_min+2*margin)/step)
        self.doses=[0.0]*(self.nx*self.nz)
        self.patch_mask=[]
        for iz in range(self.nz):
            z=self.ez_min+(iz+0.5)*step
            for ix in range(self.nx):
                x=self.ex_min+(ix+0.5)*step
                self.patch_mask.append(x_min<=x<x_max and z_min<=z<z_max)
        self.samples=0;self.weighted_seconds=0.

    def deposit(self, x,z,standoff,angle_deg,dt_s=DT_S):
        if not all(math.isfinite(v) for v in (x,z,standoff,angle_deg,dt_s)):
            raise ValueError('Non-finite deposition input')
        if standoff<=0 or dt_s<=0 or not 0<=angle_deg<=90:
            raise ValueError('Invalid virtual spray input')
        sigma=SIGMA_AT_250MM*(standoff/NOMINAL_STANDOFF)
        radius=SUPPORT_SIGMAS*sigma
        # relative flux reduction at standoff and off-normal alignment
        amp=dt_s*math.cos(math.radians(angle_deg))**2*(NOMINAL_STANDOFF/standoff)**2
        imin=max(0,int(math.floor((x-radius-self.ex_min)/self.step)))
        imax=min(self.nx-1,int(math.ceil((x+radius-self.ex_min)/self.step)))
        jmin=max(0,int(math.floor((z-radius-self.ez_min)/self.step)))
        jmax=min(self.nz-1,int(math.ceil((z+radius-self.ez_min)/self.step)))
        rad2=radius*radius; inv2s=0.5/(sigma*sigma)
        for j in range(jmin,jmax+1):
            dz=self.ez_min+(j+.5)*self.step-z
            for i in range(imin,imax+1):
                dx=self.ex_min+(i+.5)*self.step-x
                r2=dx*dx+dz*dz
                if r2<=rad2:
                    self.doses[j*self.nx+i]+=amp*math.exp(-r2*inv2s)
        self.samples+=1;self.weighted_seconds+=amp

    def rows(self,station):
        for j in range(self.nz):
            z=self.ez_min+(j+.5)*self.step
            for i in range(self.nx):
                k=j*self.nx+i
                yield (station,i,j,self.ex_min+(i+.5)*self.step,z,
                       'patch' if self.patch_mask[k] else 'overspray_apron', self.doses[k])

    def summary(self):
        patch=[v for v,is_patch in zip(self.doses,self.patch_mask) if is_patch]
        inside=sum(patch)
        outside=sum(v for v,flag in zip(self.doses,self.patch_mask) if not flag)
        mean=statistics.fmean(patch) if patch else 0.
        std=statistics.pstdev(patch) if len(patch)>1 else 0.
        return {
            'samples':self.samples,'patch_cells':len(patch),
            'dose_mean_index':mean,'dose_std_index':std,
            'dose_cv':std/mean if mean>0 else None,
            'dose_p10_index':quantile(patch,.10),'dose_p90_index':quantile(patch,.90),
            'dose_min_index':min(patch) if patch else 0.,
            'dose_max_index':max(patch) if patch else 0.,
            'relative_20pct_coverage_percent':100*sum(v>=.2*mean and v>0 for v in patch)/len(patch) if patch else 0.,
            'overspray_apron_fraction_percent':100*outside/(inside+outside) if inside+outside>0 else None,
            'inside_dose_sum':inside,'apron_dose_sum':outside,
        }
