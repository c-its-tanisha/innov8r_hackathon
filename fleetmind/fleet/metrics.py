"""Run metrics + CSV. Numbers come only from the simulation state and the task list. Nothing is invented."""
import csv
import os

COLUMNS = ['run_id', 'seed', 'planner_mode', 'allocator_mode', 'ticks', 'tasks_total', 'tasks_done',
           'avg_delivery_ticks', 'avg_urgent_delivery_ticks', 'total_distance', 'utilization_pct',
           'collisions', 'deadlocks', 'reroutes', 'waits', 'battery_used']


class RunMetrics:
    """Counts robot-ticks spent moving or carrying, and battery drained."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.robot_ticks = 0
        self.busy_ticks = 0
        self.battery_used = 0.0
        self._prev = {}
        self._last_tick = None

    def update(self, state, carrying_ids):
        tick = state['tick']
        if tick == self._last_tick:          # idempotent: one update per tick
            return
        self._last_tick = tick
        for r in state['robots']:
            rid = r['id']
            self.robot_ticks += 1
            if r['status'] == 'moving' or rid in carrying_ids:
                self.busy_ticks += 1
            prev = self._prev.get(rid)
            if prev is not None and r['battery'] < prev:
                self.battery_used += prev - r['battery']
            self._prev[rid] = r['battery']


def _avg(values):
    return round(sum(values) / len(values), 2) if values else None


def summarize(tasks, sim_metrics, tick, rm):
    """tasks: list[Task]. avg_* are over COMPLETED tasks only, so always read them next to tasks_done."""
    done = [t for t in tasks if t.status == 'done']
    lat = [t.done_tick - t.created_tick for t in done]
    urgent = [t.done_tick - t.created_tick for t in done if t.priority >= 3]
    sm = sim_metrics or {}
    util = round(100.0 * rm.busy_ticks / rm.robot_ticks, 1) if rm.robot_ticks else None
    return {
        'ticks': tick,
        'tasks_total': len(tasks),
        'tasks_done': len(done),
        'tasks_pending': sum(1 for t in tasks if t.status == 'pending'),
        'tasks_active': sum(1 for t in tasks if t.status in ('assigned', 'picked')),
        'avg_delivery_ticks': _avg(lat),
        'avg_urgent_delivery_ticks': _avg(urgent),
        'total_distance': round(sum((sm.get('distance') or {}).values()), 1),
        'utilization_pct': util,
        'collisions': sm.get('collisions', 0),
        'deadlocks': sm.get('deadlocks', 0),
        'reroutes': sm.get('reroutes', 0),
        'waits': sm.get('waits', 0),
        'battery_used': round(rm.battery_used, 1),
    }


def make_row(run_id, seed, planner_mode, allocator_mode, kpi):
    row = {'run_id': run_id, 'seed': seed, 'planner_mode': planner_mode, 'allocator_mode': allocator_mode}
    for col in COLUMNS[4:]:
        row[col] = kpi.get(col)
    return row


def append_row(path, row):
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    new = not os.path.exists(path) or os.path.getsize(path) == 0
    with open(path, 'a', newline='') as f:
        w = csv.DictWriter(f, fieldnames=COLUMNS)
        if new:
            w.writeheader()
        w.writerow({c: ('' if row.get(c) is None else row.get(c)) for c in COLUMNS})
