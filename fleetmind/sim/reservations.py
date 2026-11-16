# sim/reservations.py (Person A) - the shared "booking sheet" for robots.
# A robot books (x, y, t) = "I will stand on cell (x, y) at tick t".
# It also books each move, so two robots can never swap places through the same edge.
# A robot that finishes its path is "parked": its last cell stays booked from then on.
from collections import defaultdict


class Reservations:
    def __init__(self):
        self.vertex = {}                  # (x, y, t) -> owner
        self.edge = {}                    # (x1, y1, x2, y2, t) -> owner; move (x1,y1)@t -> (x2,y2)@t+1
        self.parked = {}                  # owner -> (x, y, t_from)
        self.last_at = defaultdict(dict)  # (x, y) -> {owner: latest tick booked on that cell}
        self.owned = defaultdict(lambda: {"v": [], "e": []})

    # ---- questions the planner asks ----
    def vertex_free(self, x, y, t, owner):
        other = self.vertex.get((x, y, t))
        if other is not None and other != owner:
            return False
        for o, (px, py, pt) in self.parked.items():
            if o != owner and px == x and py == y and t >= pt:
                return False
        return True

    def edge_free(self, x1, y1, x2, y2, t, owner):
        """Moving (x1,y1)->(x2,y2) during t -> t+1 is blocked if someone books the reverse move."""
        other = self.edge.get((x2, y2, x1, y1, t))
        return other is None or other == owner

    def goal_safe(self, x, y, t, owner):
        """Can a robot arriving at (x, y) at tick t stay there forever?"""
        for o, (px, py, _) in self.parked.items():
            if o != owner and px == x and py == y:
                return False
        for o, tmax in self.last_at.get((x, y), {}).items():
            if o != owner and tmax >= t:
                return False
        return True

    # ---- booking ----
    def reserve_path(self, owner, path, park_at_end=True):
        """path = [(x, y, t), ...] with t increasing by 1 each step."""
        self.release(owner)
        for i, (x, y, t) in enumerate(path):
            self.vertex[(x, y, t)] = owner
            self.owned[owner]["v"].append((x, y, t))
            self.last_at[(x, y)][owner] = max(self.last_at[(x, y)].get(owner, -1), t)
            if i + 1 < len(path):
                nx, ny, _ = path[i + 1]
                if (nx, ny) != (x, y):
                    self.edge[(x, y, nx, ny, t)] = owner
                    self.owned[owner]["e"].append((x, y, nx, ny, t))
        if park_at_end and path:
            x, y, t = path[-1]
            self.parked[owner] = (x, y, t)

    def park(self, owner, x, y, t):
        """Robot is standing still at (x, y) from tick t onward."""
        self.release(owner)
        self.parked[owner] = (x, y, t)

    def release(self, owner):
        for k in self.owned[owner]["v"]:
            self.vertex.pop(k, None)
        for k in self.owned[owner]["e"]:
            self.edge.pop(k, None)
        for cell in list(self.last_at):
            self.last_at[cell].pop(owner, None)
        self.owned[owner] = {"v": [], "e": []}
        self.parked.pop(owner, None)

    def prune(self, now):
        """Forget bookings in the past (keeps the sheet small)."""
        for k in [k for k in self.vertex if k[2] < now - 1]:
            del self.vertex[k]
        for k in [k for k in self.edge if k[4] < now - 1]:
            del self.edge[k]
