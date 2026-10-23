import csv, runpy, tempfile, unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
api=runpy.run_path(str(ROOT/'tools'/'audit_cell_interlock.py'))
class InterlockTests(unittest.TestCase):
    def test_stage21_and_stage22_worlds_present(self):
        self.assertEqual(len(list((ROOT/'worlds').glob('*.wbt'))),10)  # six Stage21/22 + two standard Stage23 + two dynamics experiments
    def test_only_intended_rail_has_injection(self):
        for side in ('near','far'):
            p=ROOT/'worlds'/f'CarPaintingCell_Stage21_InjectStop_{side.capitalize()}.wbt'
            s=p.read_text()
            self.assertIn(f'controller "stage21_{side}_rail"\n  controllerArgs [ "--quick" "--inject-cell-stop" ]',s)
            other='near' if side=='far' else 'far'
            self.assertIn(f'controller "stage21_{other}_rail"\n  controllerArgs [ "--quick" ]',s)
    def test_baseline_worlds_do_not_inject(self):
        for world in ('CarPaintingCell_Stage21_DualRail_Full.wbt','CarPaintingCell_Stage21_DualRail_Quick.wbt'):
            self.assertNotIn('--inject-cell-stop',(ROOT/'worlds'/world).read_text())
    def test_injection_trigger_and_stop_broadcast(self):
        for side in ('near','far'):
            rail=(ROOT/'controllers'/f'stage21_{side}_rail'/f'stage21_{side}_rail.py').read_text()
            self.assertIn('if inject_cell_stop and ack_time is not None',rail)
            self.assertIn('broadcast_stop(failure)',rail)
            self.assertIn('check_cell_stop()',rail)
    def test_analyzer_detects_injected_and_peer_stop(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            for side,reason,t in (('far','INJECTED_TEST_CELL_STOP_FROM_FAR',100.0),('near','SHARED_CELL_INTERLOCK CELL_STOP:far:INJECTED_TEST_CELL_STOP_FROM_FAR',100.064)):
                folder=root/'logs'/side;folder.mkdir(parents=True)
                with (folder/f'stage21_{side}_rail_stations.csv').open('w',newline='') as f:
                    w=csv.DictWriter(f,fieldnames=('station','status','time_s','painting_targets_passed'))
                    w.writeheader();w.writerow(dict(station='1',status=reason,time_s=str(t),painting_targets_passed=''))
            for side,t in (('far',100.0),('near',100.064)):
                with (root/'logs'/side/f'stage21_{side}_arm_stop.csv').open('w',newline='') as f:
                    w=csv.DictWriter(f,fieldnames=('stop_time_s','reason','samples','max_joint_drift_rad','hold_status'))
                    w.writeheader();w.writerow(dict(stop_time_s=str(t),reason='RAIL_STOP',samples='32',max_joint_drift_rad='0.001',hold_status='HOLD_WITHIN_DIAGNOSTIC_LIMIT'))
            result=api['analyze'](root,'far')
            self.assertEqual(result['status'],'SIMULATED_SHARED_STOP_AND_ARM_HOLD_DIAGNOSTICS_PASSED')
            self.assertEqual(result['peer_record_lag_s'],.064)
    def test_arm_hold_is_required(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            with self.assertRaises(api['AuditError']):api['arm_hold'](root,'near',10.0)

    def test_audit_recognizes_full_baseline_and_gives_actionable_instructions(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);folder=root/'logs'/'far';folder.mkdir(parents=True)
            with (folder/'stage21_far_rail_stations.csv').open('w',newline='') as f:
                w=csv.DictWriter(f,fieldnames=('station','status','time_s','painting_targets_passed'))
                w.writeheader()
                for i in range(1,4):
                    w.writerow(dict(station=str(i),status='PASS',time_s=str(i*30),painting_targets_passed='15'))
            with self.assertRaisesRegex(api['AuditError'],'normal FULL painting run'):
                api['load'](root,'far')

    def test_audit_rejects_quick_baseline_even_if_one_station(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);folder=root/'logs'/'far';folder.mkdir(parents=True)
            with (folder/'stage21_far_rail_stations.csv').open('w',newline='') as f:
                w=csv.DictWriter(f,fieldnames=('station','status','time_s','painting_targets_passed'))
                w.writeheader();w.writerow(dict(station='1',status='PASS',time_s='50',painting_targets_passed='15'))
            with self.assertRaisesRegex(api['AuditError'],'normal Quick baseline PASS'):
                api['load'](root,'far')

    def test_analyzer_rejects_false_pass(self):
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(api['AuditError']):api['analyze'](Path(tmp),'near')
if __name__=='__main__':unittest.main()
