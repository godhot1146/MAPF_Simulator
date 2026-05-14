import heapq
import math

from astar import astar


# ── Obstacle clearance ─────────────────────────────────────────────────────────

def compute_safe_positions(grid, agent_radius):
    """
    Return the set of grid cells (c, r) where an agent of given radius
    has no obstacle overlap.

    Model: the agent is a circle of radius `agent_radius` (in cells).
    A cell (c', r') is an obstacle if grid.is_obstacle(c', r').
    Position (c, r) is safe iff every cell whose center is within
    `agent_radius` of (c, r) is free.
    """
    r_ceil = int(math.ceil(agent_radius))
    safe = set()
    for row in range(grid.height):
        for col in range(grid.width):
            ok = True
            for dr in range(-r_ceil, r_ceil + 1):
                for dc in range(-r_ceil, r_ceil + 1):
                    if math.sqrt(dc * dc + dr * dr) <= agent_radius:
                        if grid.is_obstacle(col + dc, row + dr):
                            ok = False
                            break
                if not ok:
                    break
            if ok:
                safe.add((col, row))
    return safe


# ── Conflict types ─────────────────────────────────────────────────────────────

class VertexConflict:
    def __init__(self, a1, a2, col, row, time):
        self.a1, self.a2 = a1, a2
        self.col, self.row, self.time = col, row, time

    def __repr__(self):
        return f"VC(A{self.a1},A{self.a2},({self.col},{self.row}),t={self.time})"


class EdgeConflict:
    def __init__(self, a1, a2, c1, r1, c2, r2, time):
        self.a1, self.a2 = a1, a2
        self.c1, self.r1 = c1, r1
        self.c2, self.r2 = c2, r2
        self.time = time

    def __repr__(self):
        return f"EC(A{self.a1},A{self.a2},({self.c1},{self.r1})->({self.c2},{self.r2}),t={self.time})"


def _dist(c1, r1, c2, r2):
    return math.sqrt((c1 - c2) ** 2 + (r1 - r2) ** 2)


def detect_first_conflict(paths, agent_radius=0.5):
    """
    Find the first conflict considering the agent's physical size.

    Two agents conflict (vertex) at time t when the Euclidean distance
    between their positions is less than 2 * agent_radius.

    Edge conflict: standard position-swap at consecutive timesteps.
    """
    conflict_dist = 2.0 * agent_radius
    aids = list(paths.keys())
    max_t = max(len(p) for p in paths.values())

    for t in range(max_t):
        # Gather positions
        pos = {aid: paths[aid][min(t, len(paths[aid]) - 1)] for aid in aids}

        # Vertex conflicts
        for i in range(len(aids)):
            for j in range(i + 1, len(aids)):
                a1, a2 = aids[i], aids[j]
                c1, r1 = pos[a1]
                c2, r2 = pos[a2]
                if _dist(c1, r1, c2, r2) < conflict_dist:
                    return VertexConflict(a1, a2, c1, r1, t)

        # Edge conflicts (position swap)
        if t > 0:
            for i in range(len(aids)):
                for j in range(i + 1, len(aids)):
                    a1, a2 = aids[i], aids[j]
                    p1, p2 = paths[a1], paths[a2]
                    cur1  = p1[min(t,     len(p1) - 1)]
                    prev1 = p1[min(t - 1, len(p1) - 1)]
                    cur2  = p2[min(t,     len(p2) - 1)]
                    prev2 = p2[min(t - 1, len(p2) - 1)]
                    if prev1 == cur2 and cur1 == prev2:
                        return EdgeConflict(a1, a2,
                                            prev1[0], prev1[1],
                                            cur1[0],  cur1[1], t)

    return None


# ── CBS node ───────────────────────────────────────────────────────────────────

class CBSNode:
    _ctr = 0

    def __init__(self):
        self.constraints = {}  # {aid: {'v': set, 'e': set}}
        self.paths = {}
        self.cost = 0
        CBSNode._ctr += 1
        self._id = CBSNode._ctr

    def __lt__(self, other):
        return (self.cost, self._id) < (other.cost, other._id)

    def copy_constraints(self):
        return {aid: {"v": set(c["v"]), "e": set(c["e"])}
                for aid, c in self.constraints.items()}


# ── CBS solver ─────────────────────────────────────────────────────────────────

def solve_cbs(grid, agents, max_time=300, agent_radius=0.5,
              stop_event=None, progress=None, max_nodes=None):
    """
    agents       : list of {'start': (col, row), 'goal': (col, row)}
    agent_radius : physical radius in cells
    stop_event   : threading.Event — set it to cancel mid-solve
    progress     : dict updated each iteration for live visualization:
                     'paths'        – current candidate paths {idx: [(c,r),...]}
                     'conflict_pos' – (col, row) of latest conflict, or None
                     'conflict_t'   – timestep of conflict
                     'nodes'        – CBS nodes expanded so far
                     'open_size'    – current open-list size
    Returns      : {agent_idx: path_list}  or  None
    """
    n = len(agents)
    if n == 0:
        return {}

    def cancelled():
        return stop_event is not None and stop_event.is_set()

    def push_progress(paths, conflict):
        if progress is None:
            return
        if conflict is not None:
            if isinstance(conflict, VertexConflict):
                cp = (conflict.col, conflict.row)
                ct = conflict.time
            else:
                cp = ((conflict.c1 + conflict.c2) // 2,
                      (conflict.r1 + conflict.r2) // 2)
                ct = conflict.time
        else:
            cp, ct = None, 0
        # Assign as one atomic dict replace so the reader always sees consistent data
        progress['paths']        = {k: list(v) for k, v in paths.items()}
        progress['conflict_pos'] = cp
        progress['conflict_t']   = ct
        progress['nodes']        = nodes_expanded
        progress['open_size']    = len(open_list)

    safe_pos = compute_safe_positions(grid, agent_radius)
    if cancelled():
        return None

    root = CBSNode()
    root.constraints = {i: {"v": set(), "e": set()} for i in range(n)}

    for i in range(n):
        if cancelled():
            return None
        path = astar(
            grid,
            agents[i]["start"], agents[i]["goal"],
            root.constraints[i]["v"],
            root.constraints[i]["e"],
            max_time,
            safe_pos,
        )
        if path is None:
            return None
        root.paths[i] = path

    root.cost = sum(len(p) for p in root.paths.values())
    open_list = [root]
    nodes_expanded = 0
    seen_path_combos = set()   # path-combination memoization

    def paths_key(paths):
        """Hash the actual path combination — catches cases where different
        constraint sets produce identical paths (and thus identical conflicts)."""
        return tuple(sorted((i, tuple(p)) for i, p in paths.items()))

    while open_list:
        if cancelled():
            if progress is not None:
                progress['terminated'] = 'cancelled'
            return None

        node = heapq.heappop(open_list)

        # Skip if we've already evaluated this exact combination of paths
        pk = paths_key(node.paths)
        if pk in seen_path_combos:
            continue
        seen_path_combos.add(pk)

        nodes_expanded += 1

        if max_nodes is not None and nodes_expanded >= max_nodes:
            if progress is not None:
                progress['terminated'] = 'node_limit'
            return None

        conflict = detect_first_conflict(node.paths, agent_radius)
        push_progress(node.paths, conflict)

        if conflict is None:
            return node.paths

        for agent in [conflict.a1, conflict.a2]:
            if cancelled():
                return None

            child = CBSNode()
            child.constraints = node.copy_constraints()

            if isinstance(conflict, VertexConflict):
                child.constraints[agent]["v"].add(
                    (conflict.col, conflict.row, conflict.time)
                )
            else:
                if agent == conflict.a1:
                    child.constraints[agent]["e"].add(
                        (conflict.c1, conflict.r1, conflict.c2, conflict.r2, conflict.time)
                    )
                else:
                    child.constraints[agent]["e"].add(
                        (conflict.c2, conflict.r2, conflict.c1, conflict.r1, conflict.time)
                    )

            child.paths = dict(node.paths)
            path = astar(
                grid,
                agents[agent]["start"], agents[agent]["goal"],
                child.constraints[agent]["v"],
                child.constraints[agent]["e"],
                max_time,
                safe_pos,
            )
            if path is None:
                continue

            child.paths[agent] = path
            child.cost = sum(len(p) for p in child.paths.values())
            heapq.heappush(open_list, child)

    # Open list exhausted → definitively no solution
    if progress is not None:
        progress['terminated'] = 'no_solution'
    return None
