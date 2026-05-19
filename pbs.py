import math
from astar import astar
from cbs import compute_safe_positions


def solve_pbs(grid, agents, max_time=300, agent_radius=0.5,
              priority_order=None, stop_event=None, progress=None):
    """
    Prioritized Planning (simplified PBS).

    Plans agents sequentially in priority order. Each agent treats all
    higher-priority agents' paths as hard space-time constraints.

    Much faster than CBS (no constraint tree), but not guaranteed optimal.
    priority_order: list of agent indices, highest priority first.
                    Defaults to [0, 1, 2, ...].

    Returns {agent_idx: path} or None if any agent has no valid path.
    """
    n = len(agents)
    if n == 0:
        return {}

    def cancelled():
        return stop_event is not None and stop_event.is_set()

    if priority_order is None:
        priority_order = list(range(n))

    safe_pos = compute_safe_positions(grid, agent_radius)
    if cancelled():
        return None

    if progress is not None:
        progress['safe_count'] = len(safe_pos)

    if len(safe_pos) == 0:
        if progress is not None:
            progress['terminated'] = 'no_safe_positions'
        return None

    conflict_dist = 2.0 * agent_radius
    r_ceil = int(math.ceil(conflict_dist - 0.001))

    paths = {}

    for rank, i in enumerate(priority_order):
        if cancelled():
            return None

        if progress is not None:
            progress['agent']  = i
            progress['rank']   = rank
            progress['total']  = n
            progress['paths']  = {k: list(v) for k, v in paths.items()}

        # Build space-time constraints from all higher-priority agents' paths
        vc = set()
        ec = set()
        for j in priority_order[:rank]:
            if j not in paths:
                continue
            p = paths[j]
            max_t = len(p)
            for t in range(max_t):
                pos_c, pos_r = p[t]
                # Body footprint: every cell within 2*radius
                for dr in range(-r_ceil, r_ceil + 1):
                    for dc in range(-r_ceil, r_ceil + 1):
                        if math.sqrt(dc * dc + dr * dr) < conflict_dist:
                            vc.add((pos_c + dc, pos_r + dr, t))
                # Treat waiting-at-goal the same way (agent stays forever)
                if t == max_t - 1:
                    for future_t in range(max_t, max_t + max_time):
                        for dr in range(-r_ceil, r_ceil + 1):
                            for dc in range(-r_ceil, r_ceil + 1):
                                if math.sqrt(dc * dc + dr * dr) < conflict_dist:
                                    vc.add((pos_c + dc, pos_r + dr, future_t))
                # Swap (edge) constraints
                if t > 0:
                    prev_c, prev_r = p[t - 1]
                    ec.add((pos_c, pos_r, prev_c, prev_r, t - 1))

        path = astar(grid, agents[i]['start'], agents[i]['goal'],
                     vc, ec, max_time, safe_pos)

        if path is None:
            if progress is not None:
                start_ok = agents[i]['start'] in safe_pos
                goal_ok  = agents[i]['goal']  in safe_pos
                progress['failed_agent']      = i
                progress['failed_start_valid'] = start_ok
                progress['failed_goal_valid']  = goal_ok
                progress['terminated'] = (
                    'start_not_safe' if not start_ok else
                    'goal_not_safe'  if not goal_ok  else
                    'no_path'
                )
            return None

        paths[i] = path

    if progress is not None:
        progress['paths'] = {k: list(v) for k, v in paths.items()}
        progress['agent'] = -1

    return paths
