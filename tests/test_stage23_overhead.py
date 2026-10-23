import sys,unittest,math
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'controllers'/'stage23_overhead'))
from surface_planner import panels,panel_point,expected_motor_targets,make_paint_shape
class Stage23Tests(unittest.TestCase):
  def test_counts(self):
    self.assertEqual({v.name:len(v.points) for v in panels(True)},dict(roof=9,hood=9,boot=6,tailgate=6))
    self.assertEqual({v.name:len(v.points) for v in panels(False)},dict(roof=48,hood=36,boot=24,tailgate=20))
  def test_kinematics_and_geometry(self):
    for quick in (True,False):
      for p in panels(quick):
        for row,col,point in p.points:
          m=expected_motor_targets(p.name,point)
          self.assertTrue(0<=m[0]<=5.90 and 0<=m[1]<=1.40 and 0<=m[2]<=1.92)
          expect=panel_point(p.name,point)
          x=-2.95+m[0]+(0.12 if p.name=='tailgate' else 0)
          y=-.70+m[1]
          z=2.50-m[2]-(0 if p.name=='tailgate' else .12)
          self.assertLess(math.dist((x,y,z),expect),1e-9)
          geom=make_paint_shape(p.name,point,'S23_TEST',expect)
          self.assertEqual(geom.count('{'),geom.count('}'))
          self.assertIn('PBRAppearance',geom)
  def test_requires_measurement(self):
    p=panels(True)[0].points[0][2]
    with self.assertRaises(ValueError):make_paint_shape('roof',p,'S23_TEST')
    with self.assertRaises(ValueError):make_paint_shape('roof',p,'S23_TEST',(0,0,0))
  def test_world_contains_painter_and_arms(self):
    for mode in ('Quick','Full'):
      w=(ROOT/'worlds'/f'CarPaintingCell_Stage23_AllSurfaces_{mode}.wbt').read_text()
      self.assertIn('DEF STAGE23_OVERHEAD_GANTRY Robot',w)
      self.assertIn('DEF PAINT_ROBOT_NEAR UR5e',w)
      self.assertIn('DEF PAINT_ROBOT_FAR UR5e',w)
      self.assertIn(f'controllerArgs [ "--{mode.lower()}" ]',w)
if __name__=='__main__':unittest.main()
