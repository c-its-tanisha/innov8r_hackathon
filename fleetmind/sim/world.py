# sim/world.py (Person A) - the warehouse map.
# 0 = free floor, 1 = shelf. Blocked cells are extra temporary obstacles.
import config


class World:
    def __init__(self):
        self.w, self.h = config.W, config.H
        self.cells = [[0] * self.w for _ in range(self.h)]
        # Shelf blocks leave 1-cell-wide aisles at x = 5, 8, 11, 14
        for xs in ([3, 4], [6, 7], [9, 10], [12, 13], [15, 16]):
            for x in xs:
                for y in range(2, 9):
                    self.cells[y][x] = 1
        self.pickups = [{"id": "P1", "x": 1, "y": 3}, {"id": "P2", "x": 1, "y": 8}]
        self.drops = [{"id": "D1", "x": 18, "y": 2}, {"id": "D2", "x": 18, "y": 9}]
        self.charger = {"x": 0, "y": 11}
        self.starts = [(0, 10), (2, 11), (4, 10)]   # one start cell per robot
        self.blocked = set()                         # {(x, y), ...}

    def in_bounds(self, x, y):
        return 0 <= x < self.w and 0 <= y < self.h

    def is_free(self, x, y):
        return self.in_bounds(x, y) and self.cells[y][x] == 0 and (x, y) not in self.blocked

    def block(self, x, y):
        if self.in_bounds(x, y) and self.cells[y][x] == 0:
            self.blocked.add((x, y))

    def unblock(self, x, y):
        self.blocked.discard((x, y))

    def neighbors(self, x, y):
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = x + dx, y + dy
            if self.is_free(nx, ny):
                yield nx, ny
