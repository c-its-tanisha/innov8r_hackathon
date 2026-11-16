# sim/robot.py (Person A) - one warehouse robot's state.


class Robot:
    def __init__(self, rid, x, y, home):
        self.id = rid
        self.x, self.y = x, y
        self.home = tuple(home)          # where it waits when it has nothing to do
        self.heading = "E"               # N/E/S/W  (N = towards y = 0)
        self.battery = 100.0             # percent
        self.status = "idle"             # idle | moving | waiting | charging | dead
        self.goal = None                 # (x, y) or None
        self.purpose = None              # pickup | dropoff | charge | park
        self.task_id = None
        self.path = []                   # remaining [(x, y, t)]; path[0] is where it is now
        self.distance = 0                # cells travelled
        self.stuck = 0                   # consecutive ticks spent waiting with a goal
        self.idle_ticks = 0
        self.was_waiting = False

    def to_dict(self):
        return {
            "id": self.id, "x": self.x, "y": self.y, "heading": self.heading,
            "battery": round(self.battery, 1), "status": self.status,
            "goal": list(self.goal) if self.goal else None,
            "purpose": self.purpose, "task_id": self.task_id,
            "path": [[x, y, t] for (x, y, t) in self.path],
        }
