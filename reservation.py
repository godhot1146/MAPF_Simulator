"""
Reservation Table based MAPF:
1. Priority ordering
2. 2D A* for each robot (fast, C DLL)
3. Convert to timed path + insert waits at conflicts
4. Register in reservation table
"""


class ReservationTable:
    def __init__(self):
        self.table = {}  # {(c, r, t): robot_id}
        self.max_t = 0

    def clear(self):
        self.table.clear()
        self.max_t = 0

    def clear_before(self, t):
        """Remove all reservations before timestep t."""
        to_remove = [k for k in self.table if k[2] < t]
        for k in to_remove:
            del self.table[k]

    def is_reserved(self, c, r, t, exclude_id=None):
        owner = self.table.get((c, r, t))
        if owner is None:
            return False
        return owner != exclude_id

    def reserve_path(self, robot_id, timed_path, t_offset=0):
        """Register a timed path starting at t_offset."""
        for i, (c, r) in enumerate(timed_path):
            self.table[(c, r, t_offset + i)] = robot_id
        if timed_path:
            last_c, last_r = timed_path[-1]
            end_t = t_offset + len(timed_path)
            for extra_t in range(end_t, end_t + 10):
                self.table[(last_c, last_r, extra_t)] = robot_id
            self.max_t = max(self.max_t, end_t + 10)

    def unreserve(self, robot_id):
        """Remove all reservations for a robot."""
        to_remove = [k for k, v in self.table.items() if v == robot_id]
        for k in to_remove:
            del self.table[k]


def spatial_to_timed(path, speed_tpc=1):
    """Convert 2D path [(c,r),...] to timed path with speed consideration.
    tpc=1: 1 cell per timestep. tpc=2: 1 cell per 2 timesteps (insert waits)."""
    if not path:
        return []
    timed = []
    for i, pos in enumerate(path):
        timed.append(pos)
        if i < len(path) - 1 and speed_tpc > 1:
            for _ in range(speed_tpc - 1):
                timed.append(pos)
    return timed


def insert_waits(timed_path, reservation, robot_id, agent_radius=0.5, t_offset=0):
    """Insert wait steps where the path conflicts with existing reservations.
    t_offset: current global timestep (for mid-simulation replanning).
    Returns a new timed path with waits inserted."""
    if not timed_path:
        return timed_path

    result = []
    t = t_offset
    path_idx = 0
    max_waits = 30

    start_c, start_r = timed_path[0]

    while path_idx < len(timed_path):
        c, r = timed_path[path_idx]

        # Check if next position is free at time t
        next_free = not reservation.is_reserved(c, r, t, exclude_id=robot_id)

        # Also check edge conflict (swap): if we move A->B and someone else moves B->A
        if next_free and path_idx > 0 and result:
            prev_c, prev_r = result[-1]
            if reservation.is_reserved(prev_c, prev_r, t, exclude_id=robot_id):
                # Someone is moving into our previous cell — possible swap
                next_free = False

        if next_free:
            result.append((c, r))
            path_idx += 1
            t += 1
        else:
            # Wait at current position (or start position)
            if result:
                wait_pos = result[-1]
            else:
                wait_pos = timed_path[0]

            # Make sure wait position itself is not reserved
            wait_count = 0
            while wait_count < max_waits:
                if not reservation.is_reserved(c, r, t, exclude_id=robot_id):
                    break
                result.append(wait_pos)
                t += 1
                wait_count += 1

            if wait_count >= max_waits:
                result.append((c, r))
                path_idx += 1
                t += 1

    return result


def plan_all(grid, agents, safe_positions=None, agent_radius=0.5):
    """
    Plan paths for all agents using 2D A* + reservation table.

    agents: list of {'start': (c,r), 'goal': (c,r), 'tpc': int, 'priority': float}
    Returns: {agent_idx: timed_path}
    """
    from astar import astar

    reservation = ReservationTable()

    # Sort by priority (higher = plan first)
    indexed = list(enumerate(agents))
    indexed.sort(key=lambda x: x[1].get('priority', 0), reverse=True)

    results = {}

    for idx, agent in indexed:
        start = agent['start']
        goal = agent['goal']
        tpc = agent.get('tpc', 1)

        # 2D A* (will use C DLL automatically)
        path = astar(grid, start, goal, safe_positions=safe_positions)
        if path is None:
            path = [start]

        # Convert to timed path
        timed = spatial_to_timed(path, tpc)

        # Insert waits to avoid reserved cells
        timed = insert_waits(timed, reservation, idx, agent_radius)

        # Register in table
        reservation.reserve_path(idx, timed)

        results[idx] = timed

    return results, reservation
