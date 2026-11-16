# sim/engine.py (Person A) - the Simulation: the clock that ties everything together.
# Person B's FleetManager plugs in with set_controller(); it never touches robots directly.
import config
from sim.world import World
from sim.robot import Robot
from sim.reservations import Reservations
from sim.planner import plain_astar, spacetime_astar

DEADLOCK_TICKS = 20     # waiting this long with a goal = "deadlock", then replan
PARK_AFTER_IDLE = 2     # an idle robot walks home after this many idle ticks


class Simulation:
    def __init__(self, seed=42, planner_mode="spacetime"):
        self.controller = None
        self.reset(seed, planner_mode)

    # ------------------------------------------------------------ setup
    def set_controller(self, ctrl):
        """ctrl needs decide(state) -> list[command] and on_events(events)."""
        self.controller = ctrl

    def reset(self, seed=42, planner_mode="spacetime"):
        self.seed, self.planner_mode = seed, planner_mode
        self.world = World()
        self.res = Reservations()
        self.tick = 0
        self.human = None
        self.robots = []
        for i in range(config.NUM_ROBOTS):
            sx, sy = self.world.starts[i]
            self.robots.append(Robot(i + 1, sx, sy, (sx, sy)))
            if self._spacetime():
                self.res.park(i + 1, sx, sy, 0)
        self.metrics = {"collisions": 0, "deadlocks": 0, "reroutes": 0, "waits": 0}
        self._events = []
        self._prev_overlap = set()

    def _spacetime(self):
        return self.planner_mode == "spacetime"

    def _robot(self, rid):
        return next(r for r in self.robots if r.id == rid)

    # ------------------------------------------------------------ events
    def _emit(self, etype, r, **extra):
        ev = {"type": etype, "robot_id": r.id, "purpose": r.purpose,
              "task_id": r.task_id, "tick": self.tick}
        ev.update(extra)
        self._events.append(ev)

    # ------------------------------------------------------------ planning
    def _plan(self, r, goal):
        """Plan from the robot's current cell at the current tick. Returns [(x,y,t)] or None."""
        start, t0 = (r.x, r.y), self.tick
        if self._spacetime():
            self.res.release(r.id)
            path = spacetime_astar(self.world, self.res, r.id, start, goal, t0)
            if path is None:
                self.res.park(r.id, r.x, r.y, t0)      # stay put, keep the spot booked
                return None
            self.res.reserve_path(r.id, path)
            return path
        cells = plain_astar(self.world, start, goal)   # baseline: no reservations, no waiting
        if cells is None:
            return None
        return [(x, y, t0 + i) for i, (x, y) in enumerate(cells)]

    def _apply_command(self, cmd):
        r = self._robot(cmd["robot_id"])
        goal = tuple(cmd["goal"])
        r.purpose, r.task_id = cmd.get("purpose"), cmd.get("task_id")
        path = self._plan(r, goal)
        if path is None:
            self._emit("no_path", r)
            r.goal, r.path, r.status = None, [], "idle"
            r.purpose, r.task_id = None, None
            return
        r.goal, r.path = goal, path
        r.status, r.stuck, r.idle_ticks, r.was_waiting = "moving", 0, 0, False

    def _replan(self, r):
        goal = r.goal
        path = self._plan(r, goal)
        if path is None:
            self._emit("no_path", r)
            r.goal, r.path, r.status = None, [], "idle"
            r.purpose, r.task_id = None, None
            return
        r.path, r.stuck = path, 0
        self.metrics["reroutes"] += 1
        self._emit("replan", r)

    # ------------------------------------------------------------ outside controls (Person B calls these)
    def block_cell(self, x, y):
        self.world.block(x, y)
        for r in self.robots:
            if r.goal is not None and any((p[0], p[1]) == (x, y) for p in r.path[1:]):
                self._replan(r)

    def unblock_cell(self, x, y):
        self.world.unblock(x, y)

    def spawn_human(self, waypoints):
        """Not built yet (next step). Returns False so callers can ignore it safely."""
        return False

    def estimate(self, robot_id, goal):
        r = self._robot(robot_id)
        cells = plain_astar(self.world, (r.x, r.y), tuple(goal))
        if cells is None:
            return {"steps": None, "battery_needed": None}
        steps = len(cells) - 1
        return {"steps": steps, "battery_needed": round(steps * config.MOVE_BATTERY_COST, 2)}

    # ------------------------------------------------------------ the clock
    def step(self):
        """Advance the whole simulation by exactly one tick. Returns this tick's events."""
        self._events = []
        # 1. the fleet manager decides
        if self.controller is not None:
            for cmd in self.controller.decide(self.get_state()) or []:
                self._apply_command(cmd)
        # 2. time moves forward, robots execute their plans
        prev = {r.id: (r.x, r.y) for r in self.robots}
        self.tick += 1
        for r in self.robots:
            self._move(r)
        self._check_arrivals()
        self._charge()
        self._check_collisions(prev)
        self._check_deadlocks()
        self._auto_park()
        if self._spacetime():
            self.res.prune(self.tick)
        events = self._events
        if self.controller is not None:
            self.controller.on_events(events)
        return events

    def _move(self, r):
        if r.status == "dead":
            return
        if r.battery <= 0:
            r.status, r.goal, r.path = "dead", None, []
            self._emit("battery_dead", r)
            if self._spacetime():
                self.res.park(r.id, r.x, r.y, self.tick)
            return
        if not r.path:
            return
        nxt = next((p for p in r.path if p[2] == self.tick), None)
        if nxt is not None:
            nx, ny = nxt[0], nxt[1]
            if (nx, ny) != (r.x, r.y):
                dx, dy = nx - r.x, ny - r.y
                r.heading = "E" if dx > 0 else "W" if dx < 0 else "S" if dy > 0 else "N"
                r.x, r.y = nx, ny
                r.distance += 1
                r.battery = max(0.0, r.battery - config.MOVE_BATTERY_COST)
                r.status, r.stuck, r.was_waiting = "moving", 0, False
            else:
                r.status = "waiting"
                r.stuck += 1
                self.metrics["waits"] += 1
                if not r.was_waiting:
                    self._emit("wait", r)
                r.was_waiting = True
        r.path = [p for p in r.path if p[2] >= self.tick]

    def _check_arrivals(self):
        for r in self.robots:
            if r.goal is not None and (r.x, r.y) == r.goal and len(r.path) <= 1:
                self._emit("arrived", r)
                at_charge_purpose = r.purpose == "charge"
                r.goal, r.path, r.purpose, r.task_id = None, [], None, None
                r.status = "charging" if at_charge_purpose else "idle"
                r.idle_ticks, r.stuck, r.was_waiting = 0, 0, False

    def _charge(self):
        cx, cy = self.world.charger["x"], self.world.charger["y"]
        for r in self.robots:
            if r.goal is None and r.status != "dead" and (r.x, r.y) == (cx, cy):
                if r.battery < 100.0:
                    r.battery = min(100.0, r.battery + config.CHARGE_RATE)
                    r.status = "charging"
                elif r.status == "charging":
                    r.status = "idle"

    def _check_collisions(self, prev):
        cur = {r.id: (r.x, r.y) for r in self.robots}
        overlap = set()
        ids = [r.id for r in self.robots]
        for i, a in enumerate(ids):
            for b in ids[i + 1:]:
                if cur[a] == cur[b]:
                    overlap.add(("cell", a, b))
                elif cur[a] == prev[b] and cur[b] == prev[a] and cur[a] != prev[a]:
                    overlap.add(("swap", a, b))
        for kind, a, b in overlap - self._prev_overlap:
            self.metrics["collisions"] += 1
            self._emit("collision", self._robot(a), other_robot=b, kind=kind)
        self._prev_overlap = overlap

    def _check_deadlocks(self):
        for r in self.robots:
            if r.goal is not None and r.stuck >= DEADLOCK_TICKS:
                self.metrics["deadlocks"] += 1
                self._emit("deadlock", r)
                self._replan(r)

    def _auto_park(self):
        """Idle robots walk back home so they never sit in an aisle or on a station."""
        cx, cy = self.world.charger["x"], self.world.charger["y"]
        for r in self.robots:
            if r.goal is not None or r.status == "dead" or (r.x, r.y) == r.home:
                r.idle_ticks = 0 if r.goal is not None or (r.x, r.y) == r.home else r.idle_ticks
                continue
            if (r.x, r.y) == (cx, cy) and r.battery < 100.0:
                continue                                  # still charging
            r.idle_ticks += 1
            if r.idle_ticks >= PARK_AFTER_IDLE:
                saved = (r.purpose, r.task_id)
                r.purpose, r.task_id = "park", None
                path = self._plan(r, r.home)
                if path is None:
                    r.purpose, r.task_id = saved
                    r.idle_ticks = -5                     # try again in a few ticks
                else:
                    r.goal, r.path, r.status = r.home, path, "moving"
                    r.idle_ticks, r.stuck, r.was_waiting = 0, 0, False

    # ------------------------------------------------------------ the picture for the dashboard
    def get_state(self):
        w = self.world
        booked = sorted([x, y, t, o] for (x, y, t), o in self.res.vertex.items() if t >= self.tick)
        return {
            "tick": self.tick,
            "planner_mode": self.planner_mode,
            "grid": {"w": w.w, "h": w.h, "cells": w.cells},
            "stations": {"pickups": w.pickups, "drops": w.drops, "charger": w.charger},
            "blocked": [list(c) for c in sorted(w.blocked)],
            "robots": [r.to_dict() for r in self.robots],
            "human": self.human,
            "reservations": booked[:60],
            "sim_metrics": dict(self.metrics, distance={str(r.id): r.distance for r in self.robots}),
        }
