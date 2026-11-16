"""Task model + queue. Status flow: pending -> assigned -> picked -> done."""

AGING_TICKS = 50.0   # effective_priority = priority + age/50, so old low-priority tasks never starve


class Task:
    def __init__(self, tid, pickup, drop, priority, created_tick):
        self.id = tid
        self.pickup = pickup
        self.drop = drop
        self.priority = priority
        self.status = 'pending'
        self.robot_id = None
        self.created_tick = created_tick
        self.done_tick = None
        self.retry_at = 0              # do not (re)assign before this tick
        self.logged_no_robot = False   # log "no eligible robot" only once per task

    def effective_priority(self, tick):
        return self.priority + max(0, tick - self.created_tick) / AGING_TICKS

    def to_dict(self, tick=0):
        return {
            'id': self.id, 'pickup': self.pickup, 'drop': self.drop,
            'priority': self.priority, 'status': self.status, 'robot_id': self.robot_id,
            'created_tick': self.created_tick, 'done_tick': self.done_tick,
            'effective_priority': round(self.effective_priority(tick), 2),
        }


class TaskQueue:
    def __init__(self):
        self._tasks = {}
        self._next = 1

    def add(self, pickup, drop, priority, tick):
        t = Task(self._next, pickup, drop, priority, tick)
        self._tasks[t.id] = t
        self._next += 1
        return t

    def get(self, tid):
        return self._tasks.get(tid)

    def all(self):
        return [self._tasks[k] for k in sorted(self._tasks)]

    def pending(self, tick, smart=True):
        """Pending tasks that may be assigned now, in assignment order.
        smart: highest effective priority first (ties: oldest). fcfs: creation order only."""
        ready = [t for t in self._tasks.values() if t.status == 'pending' and t.retry_at <= tick]
        if smart:
            ready.sort(key=lambda t: (-t.effective_priority(tick), t.created_tick, t.id))
        else:
            ready.sort(key=lambda t: (t.created_tick, t.id))
        return ready
