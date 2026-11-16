"""FakeSimulation: same API as Person A's Simulation, so you never wait on A.
Robots walk in straight lines (x first, then y) and ignore shelves/humans. It exists only to test the
fleet logic and the dashboard. It is NOT a benchmark: never quote numbers from it."""
import random

W, H = 20, 12
PICKUPS = {'P1': (2, 2), 'P2': (2, 6), 'P3': (2, 10)}
DROPS = {'D1': (17, 2), 'D2': (17, 6), 'D3': (17, 10)}
CHARGER = (10, 11)


class FakeSimulation:
    def __init__(self, seed=42, planner_mode='spacetime'):
        self.controller = None
        self.reset(seed, planner_mode)

    def reset(self, seed=None, planner_mode=None):
        if seed is not None:
            self.seed = seed
        if planner_mode is not None:
            self.planner_mode = planner_mode
        rng = random.Random(self.seed)
        self.tick = 0
        self.blocked = set()
        self.human = None
        self.cells = [[0] * W for _ in range(H)]
        for x in (6, 7, 12, 13):
            for y in range(3, 9):
                self.cells[y][x] = 1
        starts = [(9, 11), (11, 11), (10, 10)]
        self.robots = [{'id': i + 1, 'x': x, 'y': y, 'heading': 'E', 'battery': float(rng.randint(60, 100)),
                        'status': 'idle', 'goal': None, 'purpose': None, 'task_id': None, 'path': []}
                       for i, (x, y) in enumerate(starts)]
        self.metrics = {'collisions': 0, 'deadlocks': 0, 'reroutes': 0, 'waits': 0,
                        'distance': {r['id']: 0.0 for r in self.robots}}

    def set_controller(self, ctrl):
        self.controller = ctrl

    # ----- test helper -----
    def set_robot(self, rid, **kw):
        next(r for r in self.robots if r['id'] == rid).update(kw)

    # ----- contract API -----
    def step(self):
        self.tick += 1
        if self.controller:
            for c in self.controller.decide(self.get_state()) or []:
                self._apply(c)
        events = []
        for r in self.robots:
            self._move(r, events)
        self._move_human()
        if self.controller:
            self.controller.on_events(events)
        return events

    def block_cell(self, x, y):
        self.blocked.add((x, y))

    def unblock_cell(self, x, y):
        self.blocked.discard((x, y))

    def spawn_human(self, waypoints):
        wps = [tuple(w) for w in waypoints]
        if wps:
            self.human = {'wps': wps, 'i': 1 % len(wps), 'x': wps[0][0], 'y': wps[0][1], 'vx': 0, 'vy': 0}

    def estimate(self, robot_id, goal):
        r = next(r for r in self.robots if r['id'] == robot_id)
        return abs(r['x'] - goal[0]) + abs(r['y'] - goal[1])

    def get_state(self):
        robots = []
        for r in self.robots:
            d = dict(r)
            d['goal'] = list(r['goal']) if r['goal'] else None
            d['path'] = self._path(r)
            robots.append(d)
        return {
            'tick': self.tick, 'planner_mode': self.planner_mode,
            'grid': {'w': W, 'h': H, 'cells': [row[:] for row in self.cells]},
            'stations': {'pickups': [{'id': k, 'x': v[0], 'y': v[1]} for k, v in PICKUPS.items()],
                         'drops': [{'id': k, 'x': v[0], 'y': v[1]} for k, v in DROPS.items()],
                         'charger': {'x': CHARGER[0], 'y': CHARGER[1]}},
            'blocked': [list(b) for b in sorted(self.blocked)],
            'robots': robots, 'human': self._human_state(), 'reservations': [],
            'sim_metrics': {**self.metrics, 'distance': dict(self.metrics['distance'])},
        }

    # ----- internals -----
    def _apply(self, c):
        r = next((r for r in self.robots if r['id'] == c['robot_id']), None)
        if r:
            r.update(goal=tuple(c['goal']), purpose=c['purpose'], task_id=c.get('task_id'), status='moving')

    def _move(self, r, events):
        if r['status'] == 'charging':
            r['battery'] = min(100.0, r['battery'] + 2.0)
            return
        if not r['goal']:
            return
        gx, gy = r['goal']
        if (r['x'], r['y']) != (gx, gy):
            if r['x'] != gx:
                step = 1 if gx > r['x'] else -1
                r['x'] += step
                r['heading'] = 'E' if step > 0 else 'W'
            else:
                step = 1 if gy > r['y'] else -1
                r['y'] += step
                r['heading'] = 'S' if step > 0 else 'N'
            r['battery'] = max(0.0, r['battery'] - 0.2)
            self.metrics['distance'][r['id']] += 1
        if (r['x'], r['y']) == (gx, gy):
            events.append({'type': 'arrived', 'robot_id': r['id'], 'purpose': r['purpose'],
                           'task_id': r['task_id'], 'tick': self.tick})
            r['status'] = 'charging' if r['purpose'] == 'charge' else 'idle'
            r['goal'], r['purpose'], r['task_id'] = None, None, None

    def _path(self, r):
        if not r['goal']:
            return []
        x, y, t, out = r['x'], r['y'], self.tick, []
        gx, gy = r['goal']
        while (x, y) != (gx, gy) and len(out) < 60:
            if x != gx:
                x += 1 if gx > x else -1
            else:
                y += 1 if gy > y else -1
            t += 1
            out.append([x, y, t])
        return out

    def _human_next(self, h):
        tx, ty = h['wps'][h['i']]
        x, y, i = h['x'], h['y'], h['i']
        if (x, y) == (tx, ty):
            i = (i + 1) % len(h['wps'])
            tx, ty = h['wps'][i]
        dx = (tx > x) - (tx < x)
        dy = (ty > y) - (ty < y) if dx == 0 else 0
        return x + dx, y + dy, dx, dy, i

    def _move_human(self):
        h = self.human
        if h and len(h['wps']) > 1:
            h['x'], h['y'], h['vx'], h['vy'], h['i'] = self._human_next(h)

    def _human_state(self):
        h = self.human
        if not h:
            return None
        sim, pred = dict(h), []
        for k in range(1, 5):
            if len(sim['wps']) < 2:
                break
            sim['x'], sim['y'], _, _, sim['i'] = self._human_next(sim)
            pred.append([sim['x'], sim['y'], self.tick + k])
        return {'x': h['x'], 'y': h['y'], 'vx': h['vx'], 'vy': h['vy'], 'predicted': pred}
