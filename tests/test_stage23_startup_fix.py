"""Stage23 gantry regression checks: no Webots physics implied."""
from pathlib import Path
import importlib.util
import sys
import types
import unittest

ROOT=Path(__file__).resolve().parents[1]
CONTROLLER=ROOT/'controllers'/'stage23_overhead'

class _Motor:
    def __init__(self): self.q=0.;self.target=0.
    def setPosition(self,v):self.target=v
    def getPositionSensor(self):return self
    def enable(self, *args):pass
    def getValue(self):return self.q

class _GPS:
    def __init__(self,m):self.m=m
    def enable(self,*args):pass
    def getValues(self):
        import math
        x,y,z,t=[i.q for i in self.m]
        return [-2.95+x-0.12*math.sin(t),-0.70+y,2.50-z-0.12*math.cos(t)]

class _Emitter:
    def __init__(self):self.sent=[]
    def send(self,b):self.sent.append(b)

class _Receiver:
    def enable(self,*args):pass
    def getQueueLength(self):return 0

class _Children:
    def __init__(self):self.patches=[]
    def importMFNodeFromString(self,index,geometry):self.patches.append(geometry)
class _Root:
    def __init__(self):self.children=_Children()
    def getField(self,name):return self.children

class _Simulator:
    current=None
    def __init__(self):
        self.motors=[_Motor() for _ in range(4)]
        self.devices=dict(zip(('gantry_x_motor','gantry_y_motor','gantry_z_motor','nozzle_tilt_motor'),self.motors))
        self.devices.update(stage23_nozzle_tip_gps=_GPS(self.motors),cell_stop_in=_Receiver(),cell_stop_out=_Emitter())
        self.root=_Root(); self.steps=0
        _Simulator.current=self
    def getDevice(self,name):return self.devices.get(name)
    def getRoot(self):return self.root
    def step(self,ms):
        self.steps+=1
        # Representative finite-speed motion, not ODE physics.
        for motor,vel in zip(self.motors,(0.70,0.55,0.45,1.0)):
            dx=motor.target-motor.q
            motor.q+=max(-vel*ms/1000,min(vel*ms/1000,dx))
        return 0

class _OvershootSimulator(_Simulator):
    def step(self,ms):
        out=super().step(ms)
        if self.steps==2:self.motors[2].q=3.17344997633291
        return out

class RuntimeGuards(unittest.TestCase):
    def _run(self,model):
        sys.modules['controller']=types.SimpleNamespace(Supervisor=model)
        sys.path.insert(0,str(CONTROLLER))
        name='stage23_overhead_regression'
        spec=importlib.util.spec_from_file_location(name,CONTROLLER/'stage23_overhead.py')
        mod=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        old_argv=sys.argv
        try:
            sys.argv=['stage23_overhead.py','--quick']
            mod.main()
        finally:
            sys.argv=old_argv
    def test_out_of_bounds_z_fails_closed_and_broadcasts(self):
        with self.assertRaisesRegex(RuntimeError,'GANTRY_ENCODER_OUT_OF_BOUNDS'):
            self._run(_OvershootSimulator)
        bot=_OvershootSimulator.current
        self.assertEqual(len(bot.root.children.patches),0)
        self.assertTrue(any(x.startswith(b'CELL_STOP:overhead:') for x in bot.devices['cell_stop_out'].sent))
    def test_rate_limited_motor_mock_executes_quick(self):
        self._run(_Simulator)
        bot=_Simulator.current
        self.assertEqual(len(bot.root.children.patches),30)
        self.assertGreater(bot.steps,1000)
        self.assertEqual(bot.devices['cell_stop_out'].sent,[])
    def test_default_gantry_is_kinematic_and_dynamic_variant_is_separate(self):
        for mode in ('Quick','Full'):
            a=(ROOT/'worlds'/f'CarPaintingCell_Stage23_AllSurfaces_{mode}.wbt').read_text().split('# Stage 23: motor-commanded overhead gantry')[1]
            b=(ROOT/'worlds'/f'CarPaintingCell_Stage23_GantryDynamics_Experimental_{mode}.wbt').read_text().split('# Stage 23: independent powered overhead')[1]
            self.assertNotIn('physics Physics',a)
            self.assertEqual(b.count('physics Physics'),4)
            self.assertIn('gantry_z_motor',a)
            self.assertIn('gantry_z_encoder',a)
            self.assertIn('dampingConstant 45',b)
            self.assertIn('CELL_STOP', (ROOT/'controllers'/'stage23_overhead'/'stage23_overhead.py').read_text())

if __name__=='__main__':unittest.main()
