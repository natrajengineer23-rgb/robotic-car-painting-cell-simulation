from pathlib import Path
import unittest

ROOT=Path(__file__).resolve().parents[1]

class Stage22IntegrationTests(unittest.TestCase):
    def test_worlds_contain_both_powered_rails(self):
        for mode in ('Quick','Full'):
            world=(ROOT/'worlds'/f'CarPaintingCell_Stage22_Painting_{mode}.wbt').read_text()
            for node in ('FAR_RAIL_ACTUATOR','NEAR_RAIL_ACTUATOR','PAINT_ROBOT_FAR','PAINT_ROBOT_NEAR'):
                self.assertIn(node,world)
            self.assertIn(f'"--{mode.lower()}"',world)
    def test_visible_coat_strictly_gate_controlled(self):
        for side in ('near','far'):
            path=ROOT/'controllers'/f'stage21_{side}_arm'/f'stage21_{side}_arm.py'
            arm=path.read_text()
            self.assertIn('if endpoint_credit:',arm)
            self.assertIn('paint_visual.spray_waypoint(',arm)
            self.assertIn('if quality:',arm)
            self.assertIn('paint_visual.spray_sample(',arm)
            self.assertNotIn('paint_mark(',arm)
            self.assertIn('guard_emergency(',arm)
            self.assertIn('def poll():',arm)
            self.assertIn('linkmon.guard_state',arm)

if __name__=='__main__':unittest.main()
