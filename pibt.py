import math
import random
from collections import deque


def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def bfs_dist(grid, goal, safe_pos):
    """BFS from goal: returns {pos: exact_distance} accounting for walls."""
    if goal not in safe_pos:
        return {}
    dist = {goal: 0}
    q = deque([goal])
    while q:
        pos = q.popleft()
        for nbr in grid.neighbors4(*pos):
            if nbr in safe_pos and nbr not in dist:
                dist[nbr] = dist[pos] + 1
                q.append(nbr)
    return dist


def pibt_step(grid, positions, goals, priority_order, safe_pos, agent_radius,
              distances=None):
    """
    One step of PIBT (Priority Inheritance with Backtracking).

    Each agent tries to move one cell toward its goal.
    If the target cell is occupied, the occupant inherits priority
    and must find a move first (recursive chain).

    positions     : {idx: (c, r)}  current positions
    goals         : {idx: (c, r)}  current goals
    priority_order: list of idx, highest priority first
    safe_pos      : set of valid cells

    Returns {idx: (c, r)} next positions (one step forward).
    """
    conflict_dist = 2.0 * agent_radius
    next_pos   = {}   # decided next positions
    in_progress = set()  # recursion cycle guard

    def clear_of_decided(pos, exclude):
        """True if pos doesn't conflict with any already-decided agent."""
        return all(
            math.sqrt((pos[0] - p[0]) ** 2 + (pos[1] - p[1]) ** 2) >= conflict_dist
            for j, p in next_pos.items() if j != exclude
        )

    def undecided_occupant(pos, exclude):
        """Return first undecided agent conflicting with pos, or None."""
        for j, p in positions.items():
            if j == exclude or j in next_pos:
                continue
            if math.sqrt((pos[0] - p[0]) ** 2 + (pos[1] - p[1]) ** 2) < conflict_dist:
                return j
        return None

    def do_pibt(idx):
        """Decide next cell for agent idx. Returns True on success."""
        if idx in next_pos:
            return True
        if idx in in_progress:
            return False   # cycle → signal failure to caller

        in_progress.add(idx)
        cur  = positions[idx]
        goal = goals.get(idx, cur)

        # Candidates: safe neighbors + stay, closest to goal first
        nbrs   = [p for p in grid.neighbors4(*cur) if p in safe_pos]
        # Use BFS distance if available, else fall back to Manhattan
        if distances and goal in distances:
            dist_map = distances[goal]
            key_fn = lambda p: dist_map.get(p, 10**6) + random.uniform(0, 0.4)
        else:
            key_fn = lambda p: manhattan(p, goal) + random.uniform(0, 0.4)
        cands  = sorted(nbrs + [cur], key=key_fn)

        for cand in cands:
            # Skip if already conflicts with a decided agent
            if not clear_of_decided(cand, idx):
                continue

            occ = undecided_occupant(cand, idx)

            if occ is None:
                # Cell is free → take it
                next_pos[idx] = cand
                in_progress.discard(idx)
                return True

            # Push occupant (priority inheritance)
            if do_pibt(occ) and clear_of_decided(cand, idx):
                next_pos[idx] = cand
                in_progress.discard(idx)
                return True

        # No valid move → stay put
        next_pos[idx] = cur
        in_progress.discard(idx)
        return True

    for idx in priority_order:
        do_pibt(idx)

    # Safety fill for any missed agents
    for idx in priority_order:
        if idx not in next_pos:
            next_pos[idx] = positions[idx]

    return next_pos
