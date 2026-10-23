"""Static regression tests; no substitute for Webots runtime or fault injection."""
from pathlib import Path
import runpy
import unittest
ROOT=Path(__file__).resolve().parents[1]
class GeometryTests(unittest.TestCase):
    def test_mirrored_local_grid(self):
        mod=runpy.run_path(str(ROOT/'controllers'/'stage21_near_arm'/'raster_coverage.py'))
        cl=mod['CoverageGrid']
        a=cl(-1.80,xsign=+1)
        b=cl(+2.10,xsign=-1)
        self.assertAlmostEqual(a.x_min,-1.5)
        self.assertAlmostEqual(a.x_max,-1.3)
        self.assertAlmostEqual(b.x_min,+1.6)
        self.assertAlmostEqual(b.x_max,+1.8)
        self.assertEqual(a.total,340)
        self.assertEqual(b.total,340)
    def test_two_controllers_and_separate_buses(self):
        for world in (ROOT/'worlds').glob('*.wbt'):
            t=world.read_text()
            expected=5 if 'Stage23_' in world.name else 2
            self.assertEqual(t.count('SliderJoint {'),expected)
            for c in (11,12,21,22):
                self.assertEqual(t.count(f'channel {c}'),2)
            self.assertEqual(t.count('Emitter { name "cell_stop_out" channel 33 }'),3 if 'Stage23_' in world.name else 2)
    def test_stop_is_fail_closed(self):
        for side in ('near','far'):
            arm=(ROOT/'controllers'/f'stage21_{side}_arm'/f'stage21_{side}_arm.py').read_text()
            rail=(ROOT/'controllers'/f'stage21_{side}_rail'/f'stage21_{side}_rail.py').read_text()
            self.assertIn("if data.startswith('STOP:'):guard_emergency",arm)
            self.assertIn('broadcast_stop(failure)',rail)
            self.assertIn('check_cell_stop()',rail)
            self.assertIn(f'PAINT_ROBOT_{side.upper()}',arm)
if __name__=='__main__':unittest.main()
