from pathlib import Path
import sys,types,importlib.util
p=Path(__file__).resolve().parents[1]/'controllers'/'stage23_overhead'
sys.path.insert(0,str(p))
class Sensor:
    def __init__(self,m):self.m=m
    def enable(self,*a):pass
    def getValue(self):return self.m.q
class Motor:
    def __init__(self):self.q=0
    def setPosition(self,q):self.q=q
    def getPositionSensor(self):return Sensor(self)
class Gps:
    def __init__(self,m):self.m=m
    def enable(self,*a):pass
    def getValues(self):
        import math
        x,y,z,t=[v.q for v in self.m]
        return [-2.95+x-.12*math.sin(t),-.7+y,2.5-z-.12*math.cos(t)]
class Tx:
    def __init__(self):self.messages=[]
    def send(self,msg):self.messages.append(msg)
class Rx:
    def enable(self,*a):pass
    def getQueueLength(self):return 0
class Children:
    def __init__(self):self.nodes=[]
    def importMFNodeFromString(self,idx,geom):self.nodes.append(geom)
class Root:
    def __init__(self):self.children=Children()
    def getField(self,name):return self.children
class FakeSupervisor:
    calls=[]
    def __init__(self):
        self.m=[Motor() for i in range(4)]
        self.gps=Gps(self.m)
        self.rx=Rx()
        self.tx=Tx()
        self.root=Root()
        FakeSupervisor.calls.append(self)
        self.steps=0
    def getDevice(self,name):
        names=['gantry_x_motor','gantry_y_motor','gantry_z_motor','nozzle_tilt_motor']
        if name in names:return self.m[names.index(name)]
        if name=='stage23_nozzle_tip_gps':return self.gps
        if name=='cell_stop_in':return self.rx
        if name=='cell_stop_out':return self.tx
        return None
    def getRoot(self):return self.root
    def step(self,ms):self.steps+=1;return 0
stub=types.ModuleType('controller');stub.Supervisor=FakeSupervisor;sys.modules['controller']=stub
spec=importlib.util.spec_from_file_location('stage23_test',p/'stage23_overhead.py')
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
sys.argv=['stage23_overhead.py','--quick'];a.main()
obj=FakeSupervisor.calls[-1]
assert len(obj.root.children.nodes)==30,len(obj.root.children.nodes)
print('MOCK_CONTROLLER_PASS: painted 30/30 theoretical targets, steps',obj.steps)
