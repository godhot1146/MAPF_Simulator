import heapq
import os
import time as _time
import ctypes
import numpy as np

_astar_profile = []  # list of recent slow calls for debugging

# ── Load C DLL for fast A* ────────────────────────────────────────────────────
_HAS_C = False
try:
    _dll_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "astar_c.dll")
    if os.path.exists(_dll_path):
        _cdll = ctypes.CDLL(_dll_path)
        _cdll.astar2d.restype = ctypes.c_int
        _cdll.astar2d.argtypes = [
            ctypes.POINTER(ctypes.c_ubyte), ctypes.c_int, ctypes.c_int,
            ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int,
            ctypes.POINTER(ctypes.c_int), ctypes.c_int,
        ]
        _HAS_C = True
except Exception:
    pass

_blocked_cache = (None, None)      # (grid_key, blocked_array)  base grid
_safe_blocked_cache = (None, None)  # (safe_key, blocked_array)  safe_positions version

def _astar_2d_c(grid, start, goal, safe_positions=None, on_expand=None):
    global _blocked_cache, _safe_blocked_cache
    w, h = grid.width, grid.height
    grid_key = (len(grid.obstacles), len(grid.forbidden), w, h)

    if safe_positions is not None:
        safe_key = (grid_key, len(safe_positions))
        if _safe_blocked_cache[0] != safe_key:
            blocked = np.ones((h, w), dtype=np.uint8)
            for c, r in safe_positions:
                if 0 <= c < w and 0 <= r < h:
                    blocked[r, c] = 0
            _safe_blocked_cache = (safe_key, blocked)
        blocked = _safe_blocked_cache[1]
    else:
        if _blocked_cache[0] != grid_key:
            blocked = np.zeros((h, w), dtype=np.uint8)
            for c, r in grid.obstacles:
                if 0 <= c < w and 0 <= r < h:
                    blocked[r, c] = 1
            for c, r in grid.forbidden:
                if 0 <= c < w and 0 <= r < h:
                    blocked[r, c] = 1
            _blocked_cache = (grid_key, blocked)
        blocked = _blocked_cache[1]
    max_path = w * h
    out = (ctypes.c_int * (max_path * 2))()
    n = _cdll.astar2d(
        blocked.ctypes.data_as(ctypes.POINTER(ctypes.c_ubyte)),
        w, h, start[0], start[1], goal[0], goal[1],
        out, max_path,
    )
    if n == 0:
        return None
    return [(out[i * 2], out[i * 2 + 1]) for i in range(n)]


try:
    from astar_cy import astar_2d as _astar_2d_cy
    _HAS_CYTHON = True
except ImportError:
    _HAS_CYTHON = False


def _astar_2d(grid, start, goal, safe_positions=None, on_expand=None):
    """Fast 2D A* without time dimension. Used when no constraints exist."""
    def is_valid(c, r):
        if safe_positions is not None:
            return (c, r) in safe_positions
        return grid.is_free(c, r)

    if not is_valid(*start) or not is_valid(*goal):
        return None

    open_heap = [(manhattan(start, goal), 0, start[0], start[1])]
    g_map = {start: 0}
    came_from = {}

    while open_heap:
        f, g, c, r = heapq.heappop(open_heap)
        pos = (c, r)

        if g > g_map.get(pos, float("inf")):
            continue

        if on_expand is not None:
            on_expand(c, r, 0, len(g_map))

        if pos == goal:
            path = []
            p = pos
            while p in came_from:
                path.append(p)
                p = came_from[p]
            path.append(start)
            path.reverse()
            return path

        for nc, nr in grid.neighbors4(c, r):
            if not is_valid(nc, nr):
                continue
            ng = g + 1
            npos = (nc, nr)
            if ng < g_map.get(npos, float("inf")):
                g_map[npos] = ng
                came_from[npos] = pos
                heapq.heappush(open_heap, (ng + manhattan(npos, goal), ng, nc, nr))

    return None


def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def _reconstruct(state, came_from, start):
    path = []
    s = state
    while s in came_from:
        path.append((s[0], s[1], s[2]))
        s = came_from[s]
    path.append((start[0], start[1], 0))
    path.reverse()
    return path


def expand_timed_path(timed_path):
    """Convert A* timed path [(c,r,t),...] to per-timestep positions [(c,r),...]."""
    if not timed_path:
        return []
    expanded = []
    for i in range(len(timed_path) - 1):
        c, r, t = timed_path[i]
        _, _, nt = timed_path[i + 1]
        for _ in range(nt - t):
            expanded.append((c, r))
    expanded.append((timed_path[-1][0], timed_path[-1][1]))
    return expanded


def astar(grid, start, goal, vertex_constraints=None, edge_constraints=None,
          max_time=300, safe_positions=None, partial_path=False,
          time_per_cell=1, on_expand=None):
    """
    Space-time A* with CBS constraints and optional body-clearance.

    time_per_cell : timesteps required to move one cell (default 1).
                    Use >1 for slow robots (e.g. 2 = half speed).
                    Waiting always costs 1 timestep regardless of speed.
    """
    _t0 = _time.perf_counter()

    if vertex_constraints is None:
        vertex_constraints = set()
    if edge_constraints is None:
        edge_constraints = set()

    def _profile_done(path, method):
        elapsed = _time.perf_counter() - _t0
        if elapsed > 0.1:
            entry = {
                "ms": round(elapsed * 1000, 1),
                "method": method,
                "start": start, "goal": goal,
                "v_cons": len(vertex_constraints),
                "e_cons": len(edge_constraints),
                "tpc": time_per_cell,
                "max_time": max_time,
                "path_len": len(path) if path else 0,
                "safe": "yes" if safe_positions else "no",
            }
            _astar_profile.append(entry)
            if len(_astar_profile) > 20:
                _astar_profile.pop(0)
            print(f"[A* SLOW] {entry}")

    # No constraints → fast 2D A* (skip time dimension)
    if not vertex_constraints and not edge_constraints and time_per_cell == 1:
        if _HAS_C and on_expand is None:
            result = _astar_2d_c(grid, start, goal, safe_positions)
            _profile_done(result, "C_2D")
            return result
        if _HAS_CYTHON:
            result = _astar_2d_cy(grid, start, goal, safe_positions, on_expand)
            _profile_done(result, "Cython_2D")
            return result
        result = _astar_2d(grid, start, goal, safe_positions, on_expand)
        _profile_done(result, "Python_2D")
        return result

    def is_valid(c, r):
        if safe_positions is not None:
            return (c, r) in safe_positions
        return grid.is_free(c, r)

    if not is_valid(*start) or not is_valid(*goal):
        _profile_done(None, "SpaceTime_invalid")
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

        if on_expand is not None:
            on_expand(c, r, t, len(g_map))

        if partial_path and (c, r) != goal:
            d = manhattan((c, r), goal)
            if d < best_partial_dist:
                best_partial_dist  = d
                best_partial_state = state

        if (c, r) == goal and t >= goal_free_from:
            timed = _reconstruct(state, came_from, start)
            result = expand_timed_path(timed)
            _profile_done(result, "SpaceTime")
            return result

        if t >= max_time:
            continue

        neighbors = [nb for nb in grid.neighbors4(c, r) if is_valid(*nb)]

        for nc, nr in [(c, r)] + neighbors:
            if (nc, nr) == (c, r):
                move_cost = 1          # wait: always 1 timestep
            else:
                zone_tpc = grid.speed_zones.get((nc, nr), 1)
                move_cost = max(time_per_cell, zone_tpc)
                # Slow robot stays at source during transit — check those slots
                if move_cost > 1:
                    transit_blocked = any(
                        (c, r, t + dt) in vertex_constraints
                        for dt in range(1, move_cost)
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
        timed = _reconstruct(best_partial_state, came_from, start)
        result = expand_timed_path(timed)
        _profile_done(result, "SpaceTime_partial")
        return result

    _profile_done(None, "SpaceTime_fail")
    return None
