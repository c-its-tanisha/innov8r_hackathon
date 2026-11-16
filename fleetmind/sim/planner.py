# sim/planner.py (Person A) - path planning.
# Step 1: plain A*. It ignores other robots and time.
# Step 2: Space-Time A* with reservations (so robots never collide).
import heapq


def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def plain_astar(world, start, goal):
    """Shortest path on the grid as a list of (x, y) cells, start first.
    Returns None if the goal cannot be reached."""
    start, goal = tuple(start), tuple(goal)
    if not world.is_free(*goal):
        return None
    counter = 0                                   # tie-breaker so heap never compares cells
    open_heap = [(manhattan(start, goal), counter, 0, start)]
    came_from = {start: None}
    best_g = {start: 0}
    while open_heap:
        _, _, g, cur = heapq.heappop(open_heap)
        if cur == goal:
            path = []
            while cur is not None:
                path.append(cur)
                cur = came_from[cur]
            return path[::-1]
        if g > best_g.get(cur, float("inf")):
            continue                              # stale heap entry
        for nxt in world.neighbors(*cur):
            ng = g + 1
            if ng < best_g.get(nxt, float("inf")):
                best_g[nxt] = ng
                came_from[nxt] = cur
                counter += 1
                heapq.heappush(open_heap, (ng + manhattan(nxt, goal), counter, ng, nxt))
    return None


def spacetime_astar(world, res, robot_id, start, goal, t0, horizon=150):
    """Space-Time A*: plan over (x, y, time). A robot may move to a neighbour OR wait.
    Avoids every booking in `res` (other robots, the predicted human) incl. edge swaps.
    Returns [(x, y, t), ...] starting at (start, t0), or None if no safe plan exists."""
    start, goal = tuple(start), tuple(goal)
    if not world.is_free(*goal):
        return None
    t_limit = t0 + horizon
    counter = 0
    first = (start[0], start[1], t0)
    heap = [(manhattan(start, goal), counter, first)]
    came = {first: None}
    seen = set()
    while heap:
        _, _, state = heapq.heappop(heap)
        if state in seen:
            continue
        seen.add(state)
        x, y, t = state
        if (x, y) == goal and res.goal_safe(x, y, t, robot_id):
            path = []
            while state is not None:
                path.append(state)
                state = came[state]
            return path[::-1]
        if t >= t_limit:
            continue
        for nx, ny in [(x, y)] + list(world.neighbors(x, y)):     # first option = wait
            nt = t + 1
            if not res.vertex_free(nx, ny, nt, robot_id):
                continue
            if (nx, ny) != (x, y) and not res.edge_free(x, y, nx, ny, t, robot_id):
                continue
            nxt = (nx, ny, nt)
            if nxt in came:
                continue
            came[nxt] = state
            counter += 1
            heapq.heappush(heap, ((nt - t0) + manhattan((nx, ny), goal), counter, nxt))
    return None