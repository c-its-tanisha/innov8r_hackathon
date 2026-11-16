"""Moving human obstacle with velocity prediction.

Standalone: needs only an `is_free(x, y) -> bool` callable (True if a human may
stand there). Cells are (x, y) tuples, time is in integer sim ticks.

Ground truth  : Human.step() moves the human (patrols random goals via BFS).
Observation   : the fleet only sees Human.pos each tick (observe()), and
                estimates velocity from recent positions - no cheating.
Prediction    : predicted_cells(horizon) -> {k: set(cells)} for k = 1..horizon
                ticks ahead, constant-velocity, stopping at walls, plus an
                optional safety buffer (radius in cells, Manhattan).
"""
import random
from collections import deque

DIRS = [(1, 0), (-1, 0), (0, 1), (0, -1)]


class Human:
    def __init__(self, start, is_free, width, height, speed=1, seed=None):
        """speed = ticks per cell move (1 = same speed as a robot, 2 = half)."""
        self.pos = tuple(start)
        self.is_free = is_free
        self.w, self.h = width, height
        self.speed = max(1, int(speed))
        self.rng = random.Random(seed)
        self.path = deque()
        self._tick = 0
        # observation history (what the fleet "sees")
        self.history = deque([self.pos], maxlen=4)

    # ---------- ground-truth motion ----------
    def _bfs(self, goal):
        prev, q = {self.pos: None}, deque([self.pos])
        while q:
            c = q.popleft()
            if c == goal:
                break
            for dx, dy in DIRS:
                n = (c[0] + dx, c[1] + dy)
                if n not in prev and 0 <= n[0] < self.w and 0 <= n[1] < self.h \
                        and self.is_free(*n):
                    prev[n] = c
                    q.append(n)
        if goal not in prev:
            return []
        out, c = [], goal
        while c != self.pos:
            out.append(c)
            c = prev[c]
        return out[::-1]

    def _pick_goal(self):
        for _ in range(50):
            g = (self.rng.randrange(self.w), self.rng.randrange(self.h))
            if g != self.pos and self.is_free(*g):
                p = self._bfs(g)
                if p:
                    return p
        return []

    def step(self):
        """Advance one tick. Call once per Simulation tick."""
        self._tick += 1
        if self._tick % self.speed == 0:
            if not self.path:
                self.path = deque(self._pick_goal())
            if self.path:
                self.pos = self.path.popleft()
        self.observe()

    # ---------- what the fleet sees ----------
    def observe(self):
        if not self.history or self.history[-1] != self.pos or True:
            self.history.append(self.pos)

    def velocity(self):
        """Estimated (dx, dy) per tick from the last few observations.
        Uses the most recent *actual move* so a half-speed human isn't read
        as stationary half the time."""
        h = list(self.history)
        for i in range(len(h) - 1, 0, -1):
            if h[i] != h[i - 1]:
                return (h[i][0] - h[i - 1][0], h[i][1] - h[i - 1][1])
        return (0, 0)

    def predicted_cells(self, horizon=6, buffer=0):
        """{k: set(cells)} humans' likely cells k ticks ahead (k=0 is now).
        Constant velocity at the human's move rate; stops at obstacles.
        buffer=1 also blocks the 4 neighbours of each predicted cell."""
        vx, vy = self.velocity()
        out = {}
        x, y = self.pos
        moving = (vx, vy) != (0, 0)
        for k in range(0, horizon + 1):
            if k > 0 and moving and k % self.speed == 0:
                nx, ny = x + vx, y + vy
                if 0 <= nx < self.w and 0 <= ny < self.h and self.is_free(nx, ny):
                    x, y = nx, ny
                else:
                    moving = False   # wall ahead: assume it stops/turns
            cells = {(x, y)}
            if buffer:
                for dx, dy in DIRS:
                    cells.add((x + dx, y + dy))
            out[k] = cells
        return out
