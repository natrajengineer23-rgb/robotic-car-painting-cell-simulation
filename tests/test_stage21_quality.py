"""Offline regression tests for Stage 21 dual-model instrumentation and logging."""
import csv
from pathlib import Path
import runpy
import tempfile
import unittest
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from analyze_dual_quality import analyze, ValidationError

class DepositionUnitTest(unittest.TestCase):
    def test_same_model_both_sides_and_monotonic_positive_dose(self):
        for side in ('near','far'):
            module=runpy.run_path(str(ROOT/'controllers'/f'stage21_{side}_arm'/'deposition_quality.py'))
            Dose=module['DepositionMap']
            m=Dose(-1.5,-1.3,0.68,0.85)
            s0=m.summary(); self.assertEqual(s0['samples'],0)
            m.deposit(-1.4,0.76,.25,0)
            s1=m.summary()
            self.assertEqual(s1['samples'],1)
            self.assertGreater(s1['dose_mean_index'],0)
            self.assertGreaterEqual(s1['dose_cv'],0)
            self.assertEqual(s1['patch_cells'],340)
            self.assertGreaterEqual(s1['overspray_apron_fraction_percent'],0)
            m.deposit(-1.4,0.76,.25,0)
            self.assertAlmostEqual(m.summary()['dose_mean_index'],2*s1['dose_mean_index'])

    def test_exposure_responds_to_angle_standoff(self):
        m=runpy.run_path(str(ROOT/'controllers'/'stage21_near_arm'/'deposition_quality.py'))['DepositionMap']
        a=m(-1.5,-1.3,.68,.85);b=m(-1.5,-1.3,.68,.85)
        a.deposit(-1.4,.76,.25,0)
        b.deposit(-1.4,.76,.30,10)
        self.assertLess(b.summary()['dose_max_index'],a.summary()['dose_max_index'])

    def test_gated_dose_never_enabled_during_homing_by_source(self):
        for side in ('near','far'):
            s=(ROOT/'controllers'/f'stage21_{side}_arm'/f'stage21_{side}_arm.py').read_text()
            self.assertIn('if quality:',s)
            self.assertIn('if endpoint_credit: dose.deposit(',s)
            self.assertIn('active_sweep[0]=None',s)
            self.assertIn("linkmon.guard_state=='OK'",s)
            self.assertIn("'dose_cv'",s)

class AnalyzerTest(unittest.TestCase):
    def make_fake_log(self,root, side, count):
        """Clearly synthetic *unit-test fixtures*, not user Webots results."""
        folder=root/'logs'/side;folder.mkdir(parents=True,exist_ok=True)
        def save(name,header,rows):
            with (folder/f'stage21_{side}_{name}.csv').open('w',newline='') as f:
                w=csv.writer(f);w.writerow(header);w.writerows(rows)
        save('rail_stations',['station','status','painting_targets_passed','time_s'],[[i,'PASS',15,45+i] for i in range(1,count+1)])
        save('control_waypoint_summary',['station','waypoint','status','xyz_error_m','aim_error_deg'],
             [[(i-1)//15+1,i,'POSE_PASS',.02,1.0] for i in range(1,15*count+1)])
        save('coverage_summary',['station','pose_passes','valid_virtual_spray_samples','covered_cells','total_cells','local_patch_coverage_percent'],
             [[i,15,100,340,340,100] for i in range(1,count+1)])
        save('deposition_summary',['station','pose_passes','dose_samples','patch_cells','dose_cv','relative_20pct_coverage_percent','overspray_apron_fraction_percent','dose_mean_index'],
             [[i,15,100,340,.2,100,12,.2] for i in range(1,count+1)])

    def test_good_synthetic_fixture(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d)
            for side in ('near','far'):self.make_fake_log(root,side,3)
            r=analyze(root,'full')
            self.assertEqual(r['both_sides_confirmed_raster_poses'],90)
            self.assertEqual(r['status'],'ALL_RECORDED_TESTS_PASSED')

    def test_missing_side_fails(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);self.make_fake_log(root,'near',1)
            with self.assertRaises(ValidationError):analyze(root,'quick')

    def test_partial_station_fails(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d)
            for side in ('near','far'):self.make_fake_log(root,side,1)
            p=root/'logs'/'far'/'stage21_far_rail_stations.csv'
            p.write_text(p.read_text().replace('PASS,15','PASS,14'))
            with self.assertRaises(ValidationError):analyze(root,'quick')

if __name__=='__main__':unittest.main()
