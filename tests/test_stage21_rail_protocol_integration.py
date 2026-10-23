"""Offline software integration of the UNMODIFIED Stage21 rail controller programs.

A deterministic Webots API mock supplies rail encoder/GPS/actuator and radio devices.
Tests assert cross-side stop propagation in the real Python rail loops, NOT Webots
physics, articulated arm control, or industrial safety functionality.
"""
import collections
import contextlib
import csv
import io
import sys
import tempfile
import threading
import types
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class FakeReceiver:
    def __init__(self):
        self.queue = collections.deque()
    def enable(self, ms):
        pass
    def getQueueLength(self):
        return len(self.queue)
    def getString(self):
        return self.queue[0]
    def nextPacket(self):
        self.queue.popleft()


class FakeEncoder:
    def __init__(self):
        self.position = 0.4
    def enable(self, ms):
        pass
    def getValue(self):
        return self.position


class FakeMotor:
    def __init__(self, encoder):
        self.encoder = encoder
    def setVelocity(self, velocity):
        pass
    def setPosition(self, position):
        # Fake stationary-at-command rail only; no model of motor dynamics.
        self.encoder.position = position


class FakeGPS:
    def __init__(self, side, encoder):
        self.side = side
        self.encoder = encoder
    def enable(self, ms):
        pass
    def getValues(self):
        q = self.encoder.position
        if self.side == 'far':
            return [2.50 - q, 1.55, 0.270]
        return [-2.20 + q, -1.55, 0.270]


class FakeEmitter:
    def __init__(self, world, side, channel):
        self.world = world
        self.side = side
        self.channel = channel
    def send(self, message):
        self.world.send(self.side, self.channel, message.decode('utf-8'))


class FakeRobot:
    def __init__(self, world):
        self.world = world
        self.side = threading.current_thread().name
        self.t = 0.0
        enc = FakeEncoder()
        self.devices = {
            'rail_motor': FakeMotor(enc),
            'rail_encoder': enc,
            'rail_carriage_gps': FakeGPS(self.side, enc),
            'rail_to_arm': FakeEmitter(world, self.side, 'rail_to_arm'),
            'arm_to_rail': FakeReceiver(),
            'cell_stop_out': FakeEmitter(world, self.side, 'cell_stop_out'),
            'cell_stop_in': FakeReceiver(),
        }
        world.robots[self.side] = self
    def getDevice(self, name):
        return self.devices.get(name)
    def getTime(self):
        return self.t
    def step(self, ms):
        self.t += ms / 1000.0
        if self.t == ms / 1000.0:
            self.devices['arm_to_rail'].queue.append('ARM_READY')
            self.world.first_step[self.side].set()
        try:
            self.world.barrier.wait(timeout=6)
        except threading.BrokenBarrierError as e:
            raise RuntimeError('Mock world barrier failed') from e
        return -1 if self.t >= 13.0 else 0


class FakeWorld:
    def __init__(self):
        self.barrier = threading.Barrier(2)
        self.robots = {}
        self.first_step = {s: threading.Event() for s in ('far', 'near')}
        self.emissions = []
    def send(self, source, channel, message):
        self.emissions.append((source, channel, message))
        if channel == 'rail_to_arm':
            if message.startswith('STATION:'):
                self.robots[source].devices['arm_to_rail'].queue.append('STARTED:' + message.split(':')[1])
            return
        if channel == 'cell_stop_out':
            peer = 'near' if source == 'far' else 'far'
            self.robots[peer].devices['cell_stop_in'].queue.append(message)


class RealRailProtocolTests(unittest.TestCase):
    def exercise(self, injected):
        world = FakeWorld()
        fake_module = types.ModuleType('controller')
        fake_module.Robot = lambda: FakeRobot(world)
        old_module = sys.modules.get('controller')
        old_argv = sys.argv[:]
        sys.modules['controller'] = fake_module
        errors = []
        try:
            with tempfile.TemporaryDirectory() as tmp:
                tmp_root = Path(tmp)
                modules = {}
                for side in ('far', 'near'):
                    relative = f'controllers/stage21_{side}_rail/stage21_{side}_rail.py'
                    path = tmp_root / relative
                    path.parent.mkdir(parents=True)
                    path.write_text((ROOT / relative).read_text())
                    ns = {'__file__': str(path), '__name__': 'rail_' + side}
                    exec(compile(path.read_text(), str(path), 'exec'), ns)
                    modules[side] = ns['main']

                def execute(side):
                    try:
                        # sys.argv is read only once when main() enters, before first step.
                        with contextlib.redirect_stdout(io.StringIO()):
                            modules[side]()
                    except BaseException as e:
                        errors.append((side, repr(e)))
                        world.barrier.abort()

                # Set argv for first rail, let it enter first step, then switch argv.
                # The other rail enters with the appropriate CLI arguments.
                peer = 'near' if injected == 'far' else 'far'
                sys.argv = ['rail.py', '--quick', '--inject-cell-stop']
                t_injected = threading.Thread(target=execute, args=(injected,), name=injected)
                t_injected.start()
                self.assertTrue(world.first_step[injected].wait(timeout=4))
                sys.argv = ['rail.py', '--quick']
                t_peer = threading.Thread(target=execute, args=(peer,), name=peer)
                t_peer.start()
                t_injected.join(timeout=7)
                t_peer.join(timeout=7)
                self.assertFalse(t_injected.is_alive(), 'Injected thread failed to finish')
                self.assertFalse(t_peer.is_alive(), 'Peer thread failed to finish')
                self.assertFalse(errors, errors)

                results = {}
                for side in ('near', 'far'):
                    path = tmp_root / 'logs' / side / f'stage21_{side}_rail_stations.csv'
                    with path.open(newline='') as f:
                        rows = list(csv.DictReader(f))
                    self.assertEqual(len(rows), 1, (side, rows))
                    self.assertEqual(rows[0]['station'], '1')
                    self.assertEqual(rows[0]['painting_targets_passed'], '')
                    results[side] = rows[0]

                self.assertEqual(results[injected]['status'],
                                 'INJECTED_TEST_CELL_STOP_FROM_' + injected.upper())
                self.assertEqual(results[peer]['status'],
                                 'SHARED_CELL_INTERLOCK CELL_STOP:' + injected + ':' +
                                 'INJECTED_TEST_CELL_STOP_FROM_' + injected.upper())
                self.assertTrue(any(c == 'cell_stop_out' and s == injected for s, c, m in world.emissions))
                self.assertTrue(any(c == 'rail_to_arm' and m.startswith('STOP:') for s, c, m in world.emissions if s == injected))
                self.assertTrue(any(c == 'rail_to_arm' and m.startswith('STOP:') for s, c, m in world.emissions if s == peer))
                self.assertLessEqual(abs(float(results[peer]['time_s']) - float(results[injected]['time_s'])), 0.100)
        finally:
            sys.argv = old_argv
            if old_module is None:
                sys.modules.pop('controller', None)
            else:
                sys.modules['controller'] = old_module

    def test_actual_rails_propagate_far_injection_in_fake_world(self):
        self.exercise('far')

    def test_actual_rails_propagate_near_injection_in_fake_world(self):
        self.exercise('near')
