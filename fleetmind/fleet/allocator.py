"""Cost-based task allocator (smart) and first-come-first-served allocator (baseline)."""
import math
from .battery import manhattan, is_eligible

W_D2 = 0.5      # weight on pickup->drop distance
W_BATT = 0.3    # penalty per missing battery %
W_LOAD = 3.0    # penalty per task already completed by that robot (spreads work)


class Allocator:
    def __init__(self, mode='cost', estimator=None):
        self.mode = mode
        self.estimator = estimator    # optional fn(robot_id, goal_xy) -> path length from the real sim

    def _d1(self, robot, goal):
        if self.estimator:
            try:
                v = self.estimator(robot['id'], list(goal))
                if v is not None and math.isfinite(v) and v >= 0:
                    return v
            except Exception:
                pass
        return manhattan((robot['x'], robot['y']), goal)

    def evaluate(self, robots, pickup, drop, charger, done_counts):
        """One table row per robot: this is exactly what the dashboard shows for 'why robot X'."""
        d2 = manhattan(pickup, drop)
        d3 = manhattan(drop, charger)
        rows = []
        for r in robots:
            d1 = self._d1(r, pickup)
            load = done_counts.get(r['id'], 0)
            if self.mode == 'cost':
                cost = d1 + W_D2 * d2 + W_BATT * (100 - r['battery']) + W_LOAD * load
                eligible = is_eligible(r['battery'], d1, d2, d3)
                cost = round(cost, 1)
            else:
                cost, eligible = None, True
            rows.append({'robot': r['id'], 'd1': d1, 'battery': round(r['battery'], 1),
                         'load': load, 'cost': cost, 'eligible': eligible})
        return rows

    def choose(self, task, robots, pickup, drop, charger, done_counts):
        """robots must be sorted by id. Returns (robot_id or None, table)."""
        table = self.evaluate(robots, pickup, drop, charger, done_counts)
        if not table:
            return None, table
        if self.mode == 'fcfs':
            return table[0]['robot'], table          # first idle robot by id, nothing else considered
        ok = [row for row in table if row['eligible']]
        if not ok:
            return None, table
        best = min(ok, key=lambda row: (row['cost'], str(row['robot'])))
        return best['robot'], table
