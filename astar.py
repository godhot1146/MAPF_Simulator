import heapq


def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def astar(grid, start, goal, vertex_constraints=None, edge_constraints=None,
          max_time=300, safe_positions=None):
    """
    Space-time A* with CBS constraints and optional body-clearance.

    safe_positions : precomputed set of (c, r) that have enough obstacle clearance
                     for the agent's physical radius. If None, all free cells are valid.
    """
    if vertex_constraints is None:
        vertex_constraints = set()
    if edge_constraints is None:
        edge_constraints = set()

    # Positions the agent may actually visit
    def is_valid(c, r):
        if safe_positions is not None:
            return (c, r) in safe_positions
        return grid.is_free(c, r)

    if not is_valid(*start) or not is_valid(*goal):
        return None

    # Earliest time we can safely stay at goal
    goal_free_from = 0
    for vc, vr, vt in vertex_constraints:
        if (vc, vr) == goal:
            goal_free_from = max(goal_free_from, vt + 1)

    open_heap = [(manhattan(start, goal), 0, start[0], start[1], 0)]
    g_map = {(start[0], start[1], 0): 0}
    came_from = {}

    while open_heap:
        f, g, c, r, t = heapq.heappop(open_heap)
        state = (c, r, t)

        if g > g_map.get(state, float("inf")):
            continue

        if (c, r) == goal and t >= goal_free_from:
            path = []
            s = state
            while s in came_from:
                path.append((s[0], s[1]))
                s = came_from[s]
            path.append(start)
            path.reverse()
            return path

        if t >= max_time:
            continue

        nt = t + 1
        neighbors = [nb for nb in grid.neighbors4(c, r) if is_valid(*nb)]

        for nc, nr in [(c, r)] + neighbors:
            if (nc, nr, nt) in vertex_constraints:
                continue
            if (c, r, nc, nr, t) in edge_constraints:
                continue

            nstate = (nc, nr, nt)
            ng = g + 1
            if ng < g_map.get(nstate, float("inf")):
                g_map[nstate] = ng
                came_from[nstate] = state
                heapq.heappush(open_heap, (ng + manhattan((nc, nr), goal), ng, nc, nr, nt))

    return None
