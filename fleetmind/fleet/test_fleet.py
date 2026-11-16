"""Run from the project root:  python -m unittest fleet.test_fleet -v   (no pytest needed)"""
import csv
import os
import tempfile
import unittest

from fleet import battery as bat
from fleet.allocator import Allocator
from fleet.fake_sim import FakeSimulation
from fleet.manager import FleetManager
from fleet.metrics import COLUMNS


def setup(mode='cost'):
    sim = FakeSimulation(seed=1)
    mgr = FleetManager(mode)
    sim.set_controller(mgr)
    mgr.observe(sim.get_state())
    return sim, mgr


def cmd_for(cmds, rid):
    return next((c for c in cmds if c['robot_id'] == rid), None)


class TestAllocation(unittest.TestCase):
    def test_low_battery_loses_in_allocator(self):
        robots = [{'id': 1, 'x': 10, 'y': 5, 'battery': 15.0}, {'id': 2, 'x': 5, 'y': 10, 'battery': 90.0}]
        rid, table = Allocator('cost').choose(None, robots, (10, 10), (10, 5), (10, 6), {})
        self.assertEqual(rid, 2)
        self.assertTrue(next(r for r in table if r['robot'] == 1)['cost'] > next(r for r in table if r['robot'] == 2)['cost'])

    def test_low_battery_loses_in_manager(self):
        sim, mgr = setup()
        sim.set_robot(1, battery=15.0)
        sim.set_robot(2, battery=90.0)
        sim.set_robot(3, battery=20.0)
        t = mgr.add_task('P1', 'D1', 1)
        cmds = mgr.decide(sim.get_state())
        self.assertEqual(cmd_for(cmds, 2)['purpose'], 'pickup')
        self.assertEqual(cmd_for(cmds, 2)['task_id'], t['id'])
        self.assertEqual(cmd_for(cmds, 1)['purpose'], 'charge')

    def test_cannot_finish_trip_is_ineligible(self):
        self.assertFalse(bat.is_eligible(25, 30, 20, 25))
        robots = [{'id': 1, 'x': 0, 'y': 0, 'battery': 25.0}]
        rid, table = Allocator('cost').choose(None, robots, (30, 0), (50, 0), (25, 0), {})
        self.assertIsNone(rid)
        self.assertFalse(table[0]['eligible'])

    def test_priority_3_beats_older_priority_1(self):
        sim, mgr = setup()
        sim.set_robot(1, battery=100.0)
        sim.set_robot(2, battery=20.0)
        sim.set_robot(3, battery=20.0)
        mgr.add_task('P1', 'D1', 1)
        mgr.add_task('P2', 'D2', 1)
        urgent = mgr.add_task('P3', 'D3', 3)
        cmds = mgr.decide(sim.get_state())
        self.assertEqual(cmd_for(cmds, 1)['task_id'], urgent['id'])

    def test_fcfs_ignores_distance_but_cost_does_not(self):
        for mode, expected in (('fcfs', 1), ('cost', 2)):
            sim, mgr = setup(mode)
            sim.set_robot(1, x=18, y=10, battery=100.0)    # far from P1
            sim.set_robot(2, x=2, y=3, battery=100.0)      # next to P1
            sim.set_robot(3, battery=5.0)
            mgr.add_task('P1', 'D1', 1)
            cmds = mgr.decide(sim.get_state())
            self.assertEqual(cmd_for(cmds, expected)['purpose'], 'pickup', mode)


class TestBatteryPolicy(unittest.TestCase):
    def test_idle_robot_below_35_is_sent_to_charge(self):
        sim, mgr = setup()
        sim.set_robot(1, x=5, y=5, battery=30.0)
        cmd = cmd_for(mgr.decide(sim.get_state()), 1)
        self.assertEqual(cmd['purpose'], 'charge')
        self.assertEqual(cmd['goal'], [10, 11])

    def test_baseline_only_charges_below_15(self):
        sim, mgr = setup('fcfs')
        sim.set_robot(1, x=5, y=5, battery=30.0)
        self.assertIsNone(cmd_for(mgr.decide(sim.get_state()), 1))

    def test_no_second_charge_command_while_en_route(self):
        sim, mgr = setup()
        sim.set_robot(1, x=5, y=5, battery=30.0)
        mgr.decide(sim.get_state())
        sim.set_robot(1, status='moving')
        self.assertIsNone(cmd_for(mgr.decide(sim.get_state()), 1))


class TestLifecycle(unittest.TestCase):
    def test_preemption_for_urgent_when_all_busy(self):
        sim, mgr = setup()
        for rid in (1, 2, 3):
            sim.set_robot(rid, battery=100.0)
        for p, d in (('P1', 'D1'), ('P2', 'D2'), ('P3', 'D3')):
            mgr.add_task(p, d, 1)
        for c in mgr.decide(sim.get_state()):
            sim._apply(c)
        urgent = mgr.add_task('P2', 'D3', 3)
        cmds = mgr.decide(sim.get_state())
        self.assertEqual(len(cmds), 1)
        self.assertEqual(cmds[0]['task_id'], urgent['id'])
        self.assertEqual(sum(1 for t in mgr.tasks.all() if t.status == 'pending'), 1)

    def test_no_path_returns_task_and_retries_after_5_ticks_idempotently(self):
        sim, mgr = setup()
        sim.set_robot(1, battery=100.0)
        sim.set_robot(2, battery=20.0)
        sim.set_robot(3, battery=20.0)
        t = mgr.add_task('P1', 'D1', 1)
        mgr.decide(sim.get_state())
        ev = {'type': 'no_path', 'robot_id': 1, 'purpose': 'pickup', 'task_id': t['id'], 'tick': sim.tick}
        mgr.on_events([ev])
        mgr.on_events([ev])                                    # duplicate event must change nothing
        task = mgr.tasks.get(t['id'])
        self.assertEqual(task.status, 'pending')
        self.assertEqual(task.retry_at, sim.tick + 5)
        self.assertIsNone(cmd_for(mgr.decide(sim.get_state()), 1))
        sim.tick += 5
        self.assertEqual(cmd_for(mgr.decide(sim.get_state()), 1)['task_id'], t['id'])

    def test_duplicate_arrived_event_sends_one_dropoff(self):
        sim, mgr = setup()
        sim.set_robot(1, battery=100.0)
        sim.set_robot(2, battery=20.0)
        sim.set_robot(3, battery=20.0)
        t = mgr.add_task('P1', 'D1', 1)
        mgr.decide(sim.get_state())
        ev = {'type': 'arrived', 'robot_id': 1, 'purpose': 'pickup', 'task_id': t['id'], 'tick': 3}
        mgr.on_events([ev])
        mgr.on_events([ev])
        drops = [c for c in mgr.decide(sim.get_state()) if c['purpose'] == 'dropoff']
        self.assertEqual(len(drops), 1)
        self.assertEqual(mgr.tasks.get(t['id']).status, 'picked')

    def test_end_to_end_on_fake_sim(self):
        sim, mgr = setup()
        for p, d, pr in (('P1', 'D1', 1), ('P2', 'D2', 2), ('P3', 'D3', 3), ('P1', 'D2', 1), ('P2', 'D3', 1)):
            mgr.add_task(p, d, pr)
        for _ in range(900):
            sim.step()
            if all(t.status == 'done' for t in mgr.tasks.all()):
                break
        self.assertTrue(all(t.status == 'done' for t in mgr.tasks.all()))
        self.assertEqual(mgr.kpi()['tasks_done'], 5)


class TestBenchmark(unittest.TestCase):
    def test_benchmark_writes_csv_with_right_columns(self):
        from scenarios.demo import run_benchmark, COMBOS
        with tempfile.TemporaryDirectory() as d:
            out = os.path.join(d, 'benchmark.csv')
            run_benchmark(seeds=[1], out=out, sim_cls=FakeSimulation, max_ticks=900)
            with open(out, newline='') as f:
                rows = list(csv.DictReader(f))
        self.assertEqual(list(rows[0].keys()), COLUMNS)
        self.assertEqual(len(rows), len(COMBOS))


if __name__ == '__main__':
    unittest.main()
