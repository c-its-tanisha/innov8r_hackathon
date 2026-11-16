"""FleetManager: the policy brain. Person A's sim calls decide() every tick and on_events() after movement."""
from . import battery as bat
from .allocator import Allocator
from .metrics import RunMetrics, summarize
from .tasks import TaskQueue

RETRY_TICKS = 5        # wait this long after a 'no_path' before retrying
WATCHDOG_TICKS = 8     # a busy robot idle this long gets its last command re-sent
LOG_LIMIT = 300


class FleetManager:
    def __init__(self, allocator_mode='cost', estimator=None, enable_preemption=True):
        if allocator_mode not in ('cost', 'fcfs'):
            raise ValueError("allocator_mode must be 'cost' or 'fcfs'")
        self.mode = allocator_mode
        self.allocator = Allocator(allocator_mode, estimator)
        self.enable_preemption = enable_preemption      # urgent preemption: cut first if behind
        self.tasks = TaskQueue()
        self.metrics = RunMetrics()
        self.tick = 0
        self.stations = None            # {'P1': (x, y), 'D1': (x, y)}
        self.charger = None
        self.sim_metrics = {}
        self.assignments = {}           # robot_id -> task_id
        self.charging = set()           # robots sent to charge (en route or sitting on the charger)
        self.done_counts = {}
        self.alloc_log = []
        self.event_log = []
        self._outbox = {}               # commands queued by on_events, sent at next decide()
        self._retry = {}                # robot_id -> (due_tick, command)
        self._last_cmd = {}
        self._idle_ticks = {}

    # ---------- public API (contract) ----------
    def add_task(self, pickup, drop, priority=1, tick=None):
        if self.stations is not None:
            for s in (pickup, drop):
                if s not in self.stations:
                    raise ValueError(f'unknown station {s}')
        priority = max(1, min(3, int(priority)))
        t = self.tasks.add(pickup, drop, priority, self.tick if tick is None else tick)
        self._log(t.created_tick, 'task', None, f'task #{t.id} {pickup}->{drop} priority {priority} created')
        return t.to_dict(self.tick)

    def observe(self, state):
        """Cache static info (stations, charger) and the latest tick/metrics."""
        self.tick = state['tick']
        self.sim_metrics = state.get('sim_metrics') or {}
        st = state.get('stations') or {}
        self.stations = {s['id']: (s['x'], s['y']) for s in st.get('pickups', []) + st.get('drops', [])}
        c = st.get('charger')
        self.charger = (c['x'], c['y']) if c else None

    def decide(self, state):
        self.observe(state)
        tick = self.tick
        robots = sorted(state['robots'], key=lambda r: r['id'])
        self.metrics.update(state, self._carrying())
        out = dict(self._outbox)
        self._outbox = {}

        for rid, (due, cmd) in list(self._retry.items()):          # retries after no_path
            if tick >= due:
                out[rid] = cmd
                del self._retry[rid]

        for r in robots:                                            # watchdog: stuck-idle busy robots
            rid = r['id']
            busy = rid in self.assignments or (rid in self.charging and not self._at_charger(r))
            if busy and r['status'] == 'idle' and rid not in out and rid not in self._retry:
                n = self._idle_ticks.get(rid, 0) + 1
                if n >= WATCHDOG_TICKS and rid in self._last_cmd:
                    out[rid] = self._last_cmd[rid]
                    n = 0
                    self._log(tick, 'watchdog', rid, f'robot {rid} idle too long, command re-sent')
                self._idle_ticks[rid] = n
            else:
                self._idle_ticks[rid] = 0

        if self.charger is not None:                                # battery policy
            for r in robots:
                rid = r['id']
                if rid in self.assignments or rid in self.charging or rid in out:
                    continue
                if r['status'] == 'idle' and not self._at_charger(r) and bat.needs_charge(r['battery'], self.mode):
                    out[rid] = self._cmd(rid, self.charger, 'charge', None)
                    self.charging.add(rid)
                    self._log(tick, 'charge', rid, f"robot {rid} at {r['battery']:.0f}% sent to charge")

        smart = self.mode == 'cost'
        available = [r for r in robots if r['id'] not in out and self._available(r)]
        for task in self.tasks.pending(tick, smart):                # allocation
            if not available:
                break
            rid, table = self._choose(task, available)
            if rid is None:
                if not task.logged_no_robot and table:
                    task.logged_no_robot = True
                    self._log_alloc(tick, task, None, table)
                continue
            available = [r for r in available if r['id'] != rid]
            out[rid] = self._assign(task, rid, tick, table)

        if smart and self.enable_preemption and not available:      # urgent preemption
            for task in self.tasks.pending(tick, True):
                if task.priority < 3:
                    continue
                cand = [r for r in robots if r['id'] not in out and self._preemptable(r)]
                rid, table = self._choose(task, cand) if cand else (None, [])
                if rid is None:
                    continue
                old = self.tasks.get(self.assignments.pop(rid))
                old.status, old.robot_id, old.retry_at, old.logged_no_robot = 'pending', None, 0, False
                out[rid] = self._assign(task, rid, tick, table, preempted=old.id)

        for rid, cmd in out.items():
            self._last_cmd[rid] = cmd
            self._idle_ticks[rid] = 0
        return list(out.values())

    def on_events(self, events):
        for ev in events:
            tick = ev.get('tick', self.tick)
            self.tick = max(self.tick, tick)
            typ, rid, purpose = ev.get('type'), ev.get('robot_id'), ev.get('purpose')
            tid = ev.get('task_id')
            task = self.tasks.get(tid) if tid is not None else None
            if typ == 'arrived':
                self._on_arrived(tick, rid, purpose, task)
            elif typ == 'no_path':
                self._on_no_path(tick, rid, purpose, task)
            elif typ in ('collision', 'deadlock'):
                self._log(tick, typ, rid, f'{typ} involving robot {rid}')
            elif typ == 'replan':
                self._log(tick, 'replan', rid, f'robot {rid} replanned its route', dedupe=3)

    def kpi(self):
        k = summarize(self.tasks.all(), self.sim_metrics, self.tick, self.metrics)
        k['allocator_mode'] = self.mode
        return k

    def snapshot(self):
        return {'tasks': [t.to_dict(self.tick) for t in self.tasks.all()],
                'alloc_log': self.alloc_log[-30:],
                'kpi': self.kpi(),
                'event_log': self.event_log[-40:]}

    # ---------- event handlers (idempotent) ----------
    def _on_arrived(self, tick, rid, purpose, task):
        if purpose == 'pickup':
            if task and task.status == 'assigned' and task.robot_id == rid:
                task.status = 'picked'
                self._outbox[rid] = self._cmd(rid, self.stations[task.drop], 'dropoff', task.id)
                self._log(tick, 'picked', rid, f'robot {rid} picked up task #{task.id}')
        elif purpose == 'dropoff':
            if task and task.status == 'picked' and task.robot_id == rid:
                task.status, task.done_tick = 'done', tick
                self.assignments.pop(rid, None)
                self.done_counts[rid] = self.done_counts.get(rid, 0) + 1
                self._log(tick, 'done', rid, f'robot {rid} delivered task #{task.id}')
        elif purpose == 'charge':
            self.charging.add(rid)
            self._log(tick, 'charge', rid, f'robot {rid} reached the charger')

    def _on_no_path(self, tick, rid, purpose, task):
        due = tick + RETRY_TICKS
        if purpose == 'charge' and self.charger is not None:
            self._retry[rid] = (due, self._cmd(rid, self.charger, 'charge', None))
        elif task and task.robot_id == rid and task.status == 'assigned':
            task.status, task.robot_id, task.retry_at = 'pending', None, due
            task.logged_no_robot = False
            self.assignments.pop(rid, None)
        elif task and task.robot_id == rid and task.status == 'picked':
            self._retry[rid] = (due, self._cmd(rid, self.stations[task.drop], 'dropoff', task.id))
        else:
            return
        self._log(tick, 'no_path', rid, f'robot {rid} has no path, retry in {RETRY_TICKS} ticks')

    # ---------- helpers ----------
    @staticmethod
    def _cmd(rid, goal, purpose, task_id):
        return {'robot_id': rid, 'goal': [goal[0], goal[1]], 'purpose': purpose, 'task_id': task_id}

    def _at_charger(self, r):
        return self.charger is not None and (r['x'], r['y']) == self.charger

    def _carrying(self):
        return {rid for rid, tid in self.assignments.items()
                if self.tasks.get(tid) and self.tasks.get(tid).status == 'picked'}

    def _available(self, r):
        rid = r['id']
        if rid in self.assignments or rid in self._retry:
            return False
        at = self._at_charger(r)
        if rid in self.charging and not at:
            return False                          # never assign a robot that is heading to charge
        if r['status'] == 'charging':
            return r['battery'] >= bat.RESUME_AT
        if r['status'] == 'idle':
            return (not at) or r['battery'] >= bat.RESUME_AT
        return False

    def _preemptable(self, r):
        tid = self.assignments.get(r['id'])
        t = self.tasks.get(tid) if tid is not None else None
        return bool(t and t.status == 'assigned' and t.priority < 3)

    def _choose(self, task, robots):
        p, d = self.stations.get(task.pickup), self.stations.get(task.drop)
        if p is None or d is None or self.charger is None:
            return None, []
        return self.allocator.choose(task, robots, p, d, self.charger, self.done_counts)

    def _assign(self, task, rid, tick, table, preempted=None):
        task.status, task.robot_id, task.logged_no_robot = 'assigned', rid, False
        self.assignments[rid] = task.id
        self.charging.discard(rid)
        self._log_alloc(tick, task, rid, table, preempted)
        note = f' (preempted task #{preempted})' if preempted else ''
        self._log(tick, 'assign', rid, f'task #{task.id} -> robot {rid}{note}')
        return self._cmd(rid, self.stations[task.pickup], 'pickup', task.id)

    def _log_alloc(self, tick, task, chosen, table, preempted=None):
        self.alloc_log.append({'tick': tick, 'task_id': task.id, 'pickup': task.pickup, 'drop': task.drop,
                               'priority': task.priority, 'chosen': chosen, 'table': table,
                               'mode': self.mode, 'preempted': preempted})
        del self.alloc_log[:-LOG_LIMIT]

    def _log(self, tick, typ, rid, text, dedupe=0):
        if dedupe and self.event_log:
            last = self.event_log[-1]
            if last['type'] == typ and last['robot_id'] == rid and tick - last['tick'] < dedupe:
                return
        self.event_log.append({'tick': tick, 'type': typ, 'robot_id': rid, 'text': text})
        del self.event_log[:-LOG_LIMIT]
