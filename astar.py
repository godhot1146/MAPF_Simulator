import heapq


def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def _reconstruct(state, came_from, start):
    path = []
    s = state
    while s in came_from:
        path.append((s[0], s[1]))
        s = came_from[s]
    path.append(start)
    path.reverse()
    return path


def astar(grid, start, goal, vertex_constraints=None, edge_constraints=None,
          max_time=300, safe_positions=None, partial_path=False,
          time_per_cell=1):
    """
    Space-time A* with CBS constraints and optional body-clearance.

    time_per_cell : timesteps required to move one cell (default 1).
                    Use >1 for slow robots (e.g. 2 = half speed).
                    Waiting always costs 1 timestep regardless of speed.
    """
    if vertex_constraints is None:
        vertex_constraints = set()
    if edge_constraints is None:
        edge_constraints = set()

    def is_valid(c, r):
        if safe_positions is not None:
            return (c, r) in safe_positions
        return grid.is_free(c, r)

    if not is_valid(*start) or not is_valid(*goal):
        return None

    goal_free_from = 0
    for vc, vr, vt in vertex_constraints:
        if (vc, vr) == goal:
            goal_free_from = max(goal_free_from, vt + 1)

    open_heap = [(manhattan(start, goal), 0, start[0], start[1], 0)]
    g_map     = {(start[0], start[1], 0): 0}
    came_from = {}

    best_partial_state = None
    best_partial_dist  = float("inf")

    while open_heap:
        f, g, c, r, t = heapq.heappop(open_heap)
        state = (c, r, t)

        if g > g_map.get(state, float("inf")):
            continue

        if partial_path and (c, r) != goal:
            d = manhattan((c, r), goal)
            if d < best_partial_dist:
                best_partial_dist  = d
                best_partial_state = state

        if (c, r) == goal and t >= goal_free_from:
            return _reconstruct(state, came_from, start)

        if t >= max_time:
            continue

        neighbors = [nb for nb in grid.neighbors4(c, r) if is_valid(*nb)]

        for nc, nr in [(c, r)] + neighbors:
            if (nc, nr) == (c, r):
                move_cost = 1          # wait: always 1 timestep
            else:
                move_cost = time_per_cell  # move: time_per_cell timesteps
                # Slow robot stays at source during transit — check those slots
                if time_per_cell > 1:
                    transit_blocked = any(
                        (c, r, t + dt) in vertex_constraints
                        for dt in range(1, time_per_cell)
                    )
                    if transit_blocked:
                        continue

            nt = t + move_cost
            if (nc, nr, nt) in vertex_constraints:
                continue
            if (c, r, nc, nr, t) in edge_constraints:
                continue

            nstate = (nc, nr, nt)
            ng = g + move_cost
            if ng < g_map.get(nstate, float("inf")):
                g_map[nstate] = ng
                came_from[nstate] = state
                heapq.heappush(open_heap,
                               (ng + manhattan((nc, nr), goal), ng, nc, nr, nt))

    if partial_path and best_partial_state is not None:
        return _reconstruct(best_partial_state, came_from, start)

    return None
