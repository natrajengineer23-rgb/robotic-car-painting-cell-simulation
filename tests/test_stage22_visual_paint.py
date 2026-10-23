import csv
import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path

HERE=Path(__file__).resolve().parents[1]
MODULE=HERE/'controllers'/'stage21_near_arm'/'surface_paint.py'
spec=importlib.util.spec_from_file_location('surface_paint',MODULE)
mod=importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

class FakeChildren:
    def __init__(self):self.nodes=[]
    def importMFNodeFromString(self,index,node):
        assert index==-1
        self.nodes.append(node)

class SurfacePaintTests(unittest.TestCase):
    def test_geometry_both_sides(self):
        n=mod.paint_shape(1.1,0.78,'near','waypoint',1)
        f=mod.paint_shape(1.1,0.78,'far','sample',1)
        self.assertIn('-0.95500',n)
        self.assertIn(' 0.95500',f)
        self.assertIn('Box { size',n)
        self.assertIn('Cylinder { radius',f)
        self.assertNotIn('boundingObject',n)
    def test_emitted_geometry_is_balanced_vrml(self):
        """Regression: circular spray marks must not contain an extra '}'.

        Webots importMFNodeFromString may print parse errors rather than raise,
        which allowed invalid spray events to be mistakenly logged as placed.
        """
        for side in ('near', 'far'):
            for kind in ('waypoint', 'sample'):
                node = mod.paint_shape(0.17, 0.75, side, kind, 42)
                depth = 0
                for c in node:
                    if c == '{':
                        depth += 1
                    elif c == '}':
                        depth -= 1
                    self.assertGreaterEqual(depth, 0, f'{side}/{kind}: {node}')
                self.assertEqual(depth, 0, f'{side}/{kind}: {node}')
                self.assertEqual(node.count('['), node.count(']'))
                self.assertIn('PBRAppearance { baseColor 0.035 0.320 0.880', node)
                self.assertNotIn('}}', node)

    def test_fail_closed_and_log(self):
        with tempfile.TemporaryDirectory() as tmp:
            nodes=FakeChildren()
            painter=mod.SurfacePainter(nodes,tmp,'near')
            self.assertFalse(painter.spray_waypoint(1,1,1,1.0,0.7,False))
            self.assertFalse(painter.spray_waypoint(1,1,1,3.0,0.7,True))
            self.assertTrue(painter.spray_waypoint(1,1,1,1.0,0.7,True))
            self.assertEqual(len(nodes.nodes),1)
            for i in range(mod.SAMPLE_EVERY-1):
                self.assertFalse(painter.spray_sample(i,1,1,1.0,0.7,True))
            self.assertTrue(painter.spray_sample(5,1,1,1.0,0.7,True))
            self.assertEqual(len(nodes.nodes),2)
            painter.close()
            with open(painter.log_path) as fp:
                rows=list(csv.DictReader(fp))
            self.assertEqual(len(rows),2)
            self.assertEqual({r['kind'] for r in rows},{'sample','waypoint'})
    def test_reject_invalid_scene_parameters(self):
        with self.assertRaises(ValueError):mod.paint_shape(0,0.6,'other','sample',1)
        with self.assertRaises(ValueError):mod.paint_shape(float('nan'),0.6,'near','sample',1)

if __name__=='__main__':unittest.main()
