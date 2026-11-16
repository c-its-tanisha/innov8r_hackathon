"""Seeded benchmark: 12 tasks at fixed ticks + one human route, 4 planner/allocator combos x seeds.
Run from the project root:  python -m scenarios.demo --seeds 1 2 3 4 5
Add --fake to test the script itself against fake_sim (never quote fake numbers).
Every number it reports comes from results/benchmark.csv, written by the run."""
import argparse
import os
import random
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from fleet import metrics as fm                      # noqa: E402
from fleet.manager import FleetManager               # noqa: E402

INJECT_TICKS = [0, 0, 0, 10, 20, 30, 45, 60, 75, 90, 105, 120]
URGENT_INDEXES = {5, 9}                              # two urgent (priority 3) tasks, same for every combo
COMBOS = [('spacetime', 'cost'), ('spacetime', 'fcfs'), ('basic', 'cost'), ('basic', 'fcfs')]
OUT = os.path.join('results', 'benchmark.csv')


def get_sim_class(fake=False):
    if fake or os.environ.get('SIM', 'real').lower() == 'fake':
        from fleet.fake_sim import FakeSimulation
        return FakeSimulation
    from sim.engine import Simulation
    return Simulation


def build_schedule(seed, state):
    """Same tasks for every combo of one seed, so comparisons are fair."""
    rng = random.Random(seed)
    pk = [s['id'] for s in state['stations']['pickups']]
    dr = [s['id'] for s in state['stations']['drops']]
    plan = []
    for i, tick in enumerate(INJECT_TICKS):
        prio = 3 if i in URGENT_INDEXES else rng.choice([1, 1, 2])
        plan.append((tick, rng.choice(pk), rng.choice(dr), prio))
    return plan


def run_one(sim_cls, seed, planner_mode, allocator_mode, max_ticks=1500):
    sim = sim_cls(seed=seed, planner_mode=planner_mode)
    mgr = FleetManager(allocator_mode=allocator_mode)
    sim.set_controller(mgr)
    state = sim.get_state()
    mgr.observe(state)
    plan = build_schedule(seed, state)
    grid = state['grid']
    human_tick, nxt = 30, 0
    human_done = False
    while sim.tick < max_ticks:
        while nxt < len(plan) and plan[nxt][0] <= sim.tick:
            _, p, d, prio = plan[nxt]
            mgr.add_task(p, d, prio, tick=sim.tick)
            nxt += 1
        if not human_done and sim.tick >= human_tick and hasattr(sim, 'spawn_human'):
            sim.spawn_human([[grid['w'] // 2, 1], [grid['w'] // 2, grid['h'] - 2]])
            human_done = True
        sim.step()
        if nxt == len(plan) and all(t.status == 'done' for t in mgr.tasks.all()):
            break
    mgr.observe(sim.get_state())
    run_id = f'{time.strftime("%Y%m%d-%H%M%S")}-s{seed}-{planner_mode}-{allocator_mode}'
    return fm.make_row(run_id, seed, planner_mode, allocator_mode, mgr.kpi())


def run_benchmark(seeds=(1, 2, 3, 4, 5), out=OUT, sim_cls=None, max_ticks=1500, combos=COMBOS, fresh=True):
    sim_cls = sim_cls or get_sim_class()
    if fresh and os.path.exists(out):
        os.remove(out)
    rows = []
    for seed in seeds:
        for pm, am in combos:
            row = run_one(sim_cls, seed, pm, am, max_ticks)
            fm.append_row(out, row)
            rows.append(row)
            print(f"seed {seed} {pm:9s} {am:5s} done {row['tasks_done']}/{row['tasks_total']} "
                  f"ticks {row['ticks']} collisions {row['collisions']} deadlocks {row['deadlocks']}")
    return rows


def print_summary(rows):
    cols = ['tasks_done', 'avg_delivery_ticks', 'avg_urgent_delivery_ticks', 'total_distance',
            'utilization_pct', 'collisions', 'deadlocks', 'ticks']
    print('\nMean over seeds (from the rows just written):')
    print('combo'.ljust(18) + ''.join(c[:14].rjust(16) for c in cols))
    for pm, am in COMBOS:
        sub = [r for r in rows if r['planner_mode'] == pm and r['allocator_mode'] == am]
        if not sub:
            continue
        cells = []
        for c in cols:
            vals = [r[c] for r in sub if r[c] is not None]
            cells.append(f'{sum(vals) / len(vals):.1f}'.rjust(16) if vals else '-'.rjust(16))
        print(f'{pm}/{am}'.ljust(18) + ''.join(cells))


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--seeds', type=int, nargs='+', default=[1, 2, 3, 4, 5])
    ap.add_argument('--out', default=OUT)
    ap.add_argument('--max-ticks', type=int, default=1500)
    ap.add_argument('--fake', action='store_true', help='use fake_sim (testing only)')
    ap.add_argument('--append', action='store_true', help='keep existing CSV rows')
    a = ap.parse_args()
    rows = run_benchmark(a.seeds, a.out, get_sim_class(a.fake), a.max_ticks, fresh=not a.append)
    print_summary(rows)
    print(f'\nWrote {a.out}')
