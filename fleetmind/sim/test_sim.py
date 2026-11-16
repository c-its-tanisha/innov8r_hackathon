# sim/test_sim.py (Person A) - run from the project folder:  python sim/test_sim.py
import os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from sim.world import World
from sim.planner import plain_astar, spacetime_astar
from sim.reservations import Reservations


# ---------- step 1: plain A* ----------
def test_path_exists_and_is_valid():
    w = World()
    path = plain_astar(w, (0, 10), (18, 2))
    assert path is not None, "no path found"
    assert path[0] == (0, 10) and path[-1] == (18, 2)
    for (x, y) in path:
        assert w.is_free(x, y), f"path goes through a shelf at {(x, y)}"
    for a, b in zip(path, path[1:]):
        assert abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1, "path jumps more than one cell"


def test_all_stations_reachable():
    w = World()
    for p in w.pickups:
        for d in w.drops:
            assert plain_astar(w, (0, 10), (p["x"], p["y"])) is not None
            assert plain_astar(w, (p["x"], p["y"]), (d["x"], d["y"])) is not None
    assert plain_astar(w, (0, 10), (w.charger["x"], w.charger["y"])) is not None


def test_blocked_cell_changes_route():
    w = World()
    before = plain_astar(w, (5, 1), (5, 9))
    assert before is not None
    w.block(5, 5)
    after = plain_astar(w, (5, 1), (5, 9))
    assert after is not None and (5, 5) not in after and len(after) > len(before)


def test_unreachable_goal_returns_none():
    w = World()
    assert plain_astar(w, (0, 10), (3, 4)) is None


# ---------- step 2: reservations + Space-Time A* ----------
def position_at(path, t):
    """Where is the robot at tick t? Before the path: first cell. After it: parked on the last cell."""
    if t <= path[0][2]:
        return path[0][:2]
    if t >= path[-1][2]:
        return path[-1][:2]
    return path[t - path[0][2]][:2]


def conflicts(pa, pb):
    """Return a description of the first vertex or edge-swap conflict, or None."""
    t_end = max(pa[-1][2], pb[-1][2]) + 2
    for t in range(min(pa[0][2], pb[0][2]), t_end):
        if position_at(pa, t) == position_at(pb, t):
            return f"same cell {position_at(pa, t)} at t={t}"
        if position_at(pa, t) == position_at(pb, t + 1) and position_at(pb, t) == position_at(pa, t + 1) \
                and position_at(pa, t) != position_at(pa, t + 1):
            return f"edge swap {position_at(pa, t)} <-> {position_at(pb, t)} at t={t}"
    return None


def test_single_robot_spacetime_path():
    w, res = World(), Reservations()
    p = spacetime_astar(w, res, 1, (0, 10), (18, 2), 0)
    assert p is not None and p[0] == (0, 10, 0) and p[-1][:2] == (18, 2)
    assert [c[2] for c in p] == list(range(len(p))), "time must go up by exactly 1 per step"
    plain = plain_astar(w, (0, 10), (18, 2))
    assert len(p) == len(plain), "with nobody else around it should match plain A*"


def test_reservation_blocks_vertex_and_edge():
    res = Reservations()
    res.reserve_path(1, [(1, 1, 0), (2, 1, 1)], park_at_end=False)
    assert not res.vertex_free(2, 1, 1, 2)            # same cell, same time
    assert res.vertex_free(2, 1, 2, 2)                # one tick later is fine
    assert not res.edge_free(2, 1, 1, 1, 0, 2)        # swap is blocked
    assert res.edge_free(2, 1, 1, 1, 5, 2)            # other time is fine


def test_head_on_in_open_row_no_conflict():
    w, res = World(), Reservations()
    res.park(1, 0, 0, 0); res.park(2, 19, 0, 0)
    p1 = spacetime_astar(w, res, 1, (0, 0), (18, 0), 0); res.reserve_path(1, p1)
    p2 = spacetime_astar(w, res, 2, (19, 0), (1, 0), 0)
    assert p1 and p2, "both robots must find a plan"
    assert conflicts(p1, p2) is None, conflicts(p1, p2)


def test_one_wide_aisle_resolved_by_waiting():
    w, res = World(), Reservations()
    for x in (0, 1, 2, 8, 11, 14, 17, 18, 19):
        w.block(x, 5)                                 # cut every other way through the middle: only aisle x=5 is left
    # robot 1 plans first and books the aisle; robot 2 (already standing inside it) must then yield
    p1 = spacetime_astar(w, res, 1, (5, 1), (2, 10), 0); res.reserve_path(1, p1)   # goes DOWN the aisle
    p2 = spacetime_astar(w, res, 2, (5, 8), (5, 1), 0)                              # is IN the aisle, wants to go UP
    assert p1 and p2, "both robots must find a plan"
    assert conflicts(p1, p2) is None, conflicts(p1, p2)
    straight = 8                                       # cells in the straight path (5,8) -> (5,1)
    assert len(p2) > straight, "robot 2 must back off / wait for robot 1, not push straight through"
    assert p2[-1][:2] == (5, 1), "...and still reach its goal afterwards"


def test_cannot_plan_to_a_cell_someone_is_parked_on():
    w, res = World(), Reservations()
    res.park(1, 18, 2, 0)
    assert spacetime_astar(w, res, 2, (0, 10), (18, 2), 0) is None


def test_three_robots_cross_without_conflict():
    w, res = World(), Reservations()
    jobs = [(1, (0, 10), (18, 2)), (2, (2, 11), (18, 9)), (3, (4, 10), (1, 3))]
    for rid, s, _ in jobs:
        res.park(rid, s[0], s[1], 0)
    paths = []
    for rid, s, g in jobs:
        p = spacetime_astar(w, res, rid, s, g, 0)
        assert p is not None, f"robot {rid} found no plan"
        res.reserve_path(rid, p)
        paths.append(p)
    for i in range(3):
        for j in range(i + 1, 3):
            assert conflicts(paths[i], paths[j]) is None, (i + 1, j + 1, conflicts(paths[i], paths[j]))


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("PASS", name)
    print("All Person A tests passed.")
