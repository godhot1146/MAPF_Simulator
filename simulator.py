import threading
import time
import os
import pygame

from grid import Grid
from cbs import solve_cbs

# ── Palette ────────────────────────────────────────────────────────────────────
BG            = (28,  28,  32)
GRID_LINE     = (175, 175, 180)
FREE_CELL     = (200, 200, 205)
OBS_CELL      = (38,  38,  42)
SIDEBAR_BG    = (42,  42,  52)
SIDEBAR_EDGE  = (65,  65,  85)
TEXT          = (215, 215, 225)
DIM           = (110, 110, 125)
BTN_NORMAL    = (62,  68,  98)
BTN_HOVER     = (80,  88,  128)
BTN_ACTIVE    = (95,  108, 170)
MSG_BG        = (0,   0,   0,  190)
FORBID_CELL   = (180, 40,  40)
SPEED_CELL    = (255, 180, 50)

import colorsys as _colorsys

MAX_AGENTS = 99

def agent_color(idx):
    """Visually distinct color for any agent index using golden-ratio hue spacing."""
    h = (idx * 0.618033988749895) % 1.0
    r, g, b = _colorsys.hsv_to_rgb(h, 0.78, 0.92)
    return (int(r * 255), int(g * 255), int(b * 255))

SIDEBAR_W   = 225
CELL_DEFAULT      =  8
SOLVE_TIMEOUT_DEFAULT = 30.0
CELL_MIN     =  1
CELL_MAX     = 40
BASE_SPEED   = 2.0   # m/s at tpc=1 (max speed)

MODE_OBS   = "obstacle"
MODE_ERASE = "erase"
MODE_AGNT  = "agent"
MODE_FORBID = "forbidden"
MODE_SPEED  = "speed_zone"

SUB_NONE  = 0
SUB_START = 1
SUB_GOAL  = 2


# ── Tiny button helper ─────────────────────────────────────────────────────────
class Btn:
    def __init__(self, rect, label, cb, toggle=False, tip=None):
        self.rect   = pygame.Rect(rect)
        self.label  = label
        self.cb     = cb
        self.toggle = toggle
        self.active = False
        self._hover = False
        self.tip    = tip

    def hit(self, pos):
        return self.rect.collidepoint(pos)

    def on_event(self, ev):
        if ev.type == pygame.MOUSEMOTION:
            self._hover = self.rect.collidepoint(ev.pos)
        elif ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1 and self.hit(ev.pos):
            if self.cb:
                self.cb()
            return True
        return False

    def draw(self, surf, font):
        col = BTN_ACTIVE if self.active else (BTN_HOVER if self._hover else BTN_NORMAL)
        pygame.draw.rect(surf, col, self.rect, border_radius=4)
        pygame.draw.rect(surf, (88, 92, 130), self.rect, 1, border_radius=4)
        txt = font.render(self.label, True, TEXT)
        surf.blit(txt, txt.get_rect(center=self.rect.center))


# ── Main application ───────────────────────────────────────────────────────────
class MAPFApp:
    def __init__(self, cols=40, rows=30, cell=CELL_DEFAULT, last_map=None, cfg=None):
        pygame.init()
        pygame.display.set_caption("MAPF Simulator  -  CBS")

        self.cell      = cell
        self.cam_x     = 0          # grid-area pixel offset (pan)
        self.cam_y     = 0
        self._panning  = False
        self._pan_orig = (0, 0)

        self.grid      = Grid.empty(cols, rows)
        self.agents    = []          # [{'start': (c,r)|None, 'goal': (c,r)|None}]
        self.sel       = 0           # selected agent index
        self.edit_sub  = SUB_NONE
        self.mode      = MODE_OBS

        self.paths        = {}          # {agent_idx: [(c,r),...]}
        self.sim_t        = 0.0
        self.sim_step     = 0
        self.sim_run      = False
        c = cfg or {}
        self.sim_speed    = float(c.get("sim_speed",    1.0))
        self.agent_radius = float(c.get("agent_radius", 1.5))

        self._solving       = False
        self._sol_result    = None
        self._sol_error     = ""
        self._stop_event    = threading.Event()
        self._solve_start   = 0.0
        self.solve_timeout  = float(c.get("solve_timeout", SOLVE_TIMEOUT_DEFAULT))
        self.max_nodes      = int(c.get("max_nodes", 5000))
        self._cbs_progress  = {}
        self._show_progress = True     # toggle with Tab

        self._zone_speed_tpc   = 3     # default tpc for speed zones
        self._grid_cache       = None

        self._drag_painting    = False
        self._drag_val         = True
        self._box_start        = None  # (c, r) grid cell where box drag started
        self._box_cur          = None  # (c, r) current drag end cell
        self._agent_list_scroll = 0    # top agent index shown in sidebar

        # ── Continuous / auto mode ────────────────────────────────────────────
        self._cont_mode        = False
        self._cont_replan_mode = 0     # 0=A*, 1=CBS round-trip, 2=RHCR
        self._cont_agents      = {}    # {agent_idx: {'path','local_t','goal','arrivals'}}
        self._cont_total       = 0
        self._safe_cache       = None
        self._cont_cbs_run     = False
        self._cont_cbs_pend    = False
        self._cont_cbs_res     = None
        self._cont_cbs_snap    = {}
        self._cont_cbs_frac    = {}
        self._cont_progress    = {}
        self._cont_work_gen    = 0    # incremented each replan; filters stale results
        self._pbs_priority     = []   # agent index priority order (0 = highest)
        self._pibt_prev        = {}   # {idx: (c,r)} positions before last step
        self._pibt_cur         = {}   # {idx: (c,r)} positions after last step (target)
        self._pibt_frac        = 0.0  # animation fraction 0→1 per step
        self._pibt_paths       = {}   # {idx: path} reference A* paths for visualization
        self._pibt_path_tick   = 0    # step counter for path refresh
        self._pibt_dist_cache  = {}   # {goal: {pos: dist}} BFS distance cache
        self._pibt_rank        = {}   # {idx: rank} current priority rank (0=highest)
        self._rhcr_window      = int(c.get("rhcr_window", 12))  # W: planning horizon (CBS depth)
        self._rhcr_h           = int(c.get("rhcr_h",      5))   # H: execution steps before replan (H <= W)
        self._rhcr_global_t    = 0.0   # cumulative sim steps in cont mode
        self._rhcr_last_t      = 0.0   # global_t at last RHCR replan
        self._rhcr_arrived     = set() # agents counted as arrived (waiting for replan)
        self._rhcr_consec_fail = 0     # consecutive CBS failures
        self._rhcr_first_cbs   = False # wait for first CBS before moving

        self.msg       = ""
        self.msg_ticks = 0

        # ── Collision detection ───────────────────────────────────────────────
        self._collision_count   = 0          # total collision events
        self._collision_active  = set()      # pairs currently in collision (frozenset)
        self._collision_agents  = set()      # agent indices currently colliding (for highlight)
        self._collision_log     = []         # [(timestamp_str, message), ...]
        self._collision_log_max = 30         # max entries to keep
        self._show_col_log      = True       # toggle with L key

        # ── Performance stats ─────────────────────────────────────────────────
        self._stat_fps        = 0.0
        self._stat_frame_ms   = 0.0
        self._stat_replan_ms  = 0.0
        self._stat_replan_cnt = 0
        self._stat_replan_t   = 0.0
        self._stat_replan_ps  = 0.0
        self._throughput_window = []  # timestamps of recent arrivals (last 60s)
        self._throughput        = 0.0  # tasks/min

        # ── A* async replanning ───────────────────────────────────────────────
        self._astar_pending  = {}    # {idx: goal} waiting to be replanned
        self._astar_running  = False # batch worker in progress
        self._astar_results  = {}    # {idx: new_path} from worker
        self._astar_inflight = set() # agent indices currently being planned

        # ── Simulation log ────────────────────────────────────────────
        self._sim_log        = []    # buffered log entries
        self._sim_log_t0     = 0.0   # real-time start of sim

        # ── A* visualization ──────────────────────────────────────────
        self._astar_viz       = False  # toggle with V key
        self._astar_viz_agents = {}    # {agent_idx: {(c,r): t}} per-agent expanded cells
        self._astar_viz_total = 0

        self.font   = pygame.font.SysFont("consolas", 13)
        self.font_s = pygame.font.SysFont("consolas", 11)
        self.font_b = pygame.font.SysFont("consolas", 14, bold=True)
        self.font_kr = pygame.font.SysFont("malgun gothic", 10)

        self._screen_w = self.grid.width * self.cell + SIDEBAR_W
        self._screen_h = max(self.grid.height * self.cell, 520)
        info = pygame.display.Info()
        sw = min(self._screen_w, info.current_w - 40)
        sh = min(self._screen_h, info.current_h - 80)
        self.screen = pygame.display.set_mode((sw, sh), pygame.RESIZABLE)

        self.clock  = pygame.time.Clock()
        self._btns  = []
        self._rebuild_btns()

        # Auto-load last used map
        if last_map and os.path.exists(last_map):
            try:
                grid, _ = Grid.from_pgm_yaml(last_map)
                self.grid = grid
                self._save_config(last_map)
                self._show(f"Restored: {os.path.basename(last_map)}")
                self._rebuild_btns()
            except Exception:
                pass

    TOOLBAR_H = 44

    # ── Button layout ──────────────────────────────────────────────────────────
    def _rebuild_btns(self):
        # ── Top toolbar (frequently used) ────────────────────────────
        th = self.TOOLBAR_H
        tb_h = 24
        tb_g = 4
        tx = 4

        def B(rect, label, cb, toggle=False, tip=None):
            return Btn(rect, label, cb, toggle, tip)

        self.b_obs    = B((tx, 4, 50, tb_h), "Wall",   lambda: self._set_mode(MODE_OBS),    True, tip="벽(장애물) 그리기"); tx += 54
        self.b_forbid = B((tx, 4, 60, tb_h), "Forbid", lambda: self._set_mode(MODE_FORBID), True, tip="금지 구역"); tx += 64
        self.b_speed  = B((tx, 4, 50, tb_h), "Slow",   lambda: self._set_mode(MODE_SPEED),  True, tip="감속 구역"); tx += 54
        self.b_era    = B((tx, 4, 50, tb_h), "Erase",  lambda: self._set_mode(MODE_ERASE),  True, tip="지우기"); tx += 58
        self.b_viz    = B((tx, 4, 50, tb_h), "A*Viz",  self._toggle_astar_viz, True, tip="A* 탐색 시각화 (V키)"); tx += 54

        # ── Sidebar ──────────────────────────────────────────────────
        x0 = self._grid_area_w() + 8
        w  = SIDEBAR_W - 16
        hw = w // 2 - 2
        h  = 26
        g  = 5
        y  = 32

        # ── Agent ────────────────────────────────────────────────────
        self._y_agent_label = y;                                                                 y += 14+g
        self.b_agnt  = B((x0, y, w, h), "Select Agent", lambda: self._set_mode(MODE_AGNT), True); y += h+g
        self.b_add   = B((x0,       y, hw, h), "+Agent",    self._add_agent)
        self.b_del   = B((x0+hw+4,  y, hw, h), "-Agent",    self._del_agent); y += h+g
        self.b_start = B((x0,       y, hw, h), "Set Start", self._toggle_start, True)
        self.b_goal  = B((x0+hw+4,  y, hw, h), "Set Goal",  self._toggle_goal,  True); y += h+g
        self.b_rand  = B((x0, y, w, h), "Random Agents",    self._random_agents); y += h+g*2

        # ── Parameters ───────────────────────────────────────────────
        self._y_param_label = y;                                                                 y += 14+g
        self._y_radius = y;                                                                 y += 30+g
        self._y_speed = y;                                                                  y += 30+g
        self._y_slow_tpc = y;                                                               y += 18+g
        y += 18  # Timeout row
        y += 18+g  # Max-nodes row

        indent = 16

        # ── Research (comparison) ────────────────────────────────────
        self._y_research_label = y;                                                             y += 14+g
        self.b_auto_cbs   = B((x0, y, w, h), "CBS",  lambda: self._start_auto(1), True); y += h+g
        self.b_auto_rhcr  = B((x0+indent, y, w-indent, h), "RHCR", lambda: self._start_auto(2), True); y += h+g
        self._y_rhcr_wh = y;                                                                        y += 18+18+g
        self.b_auto_pbs   = B((x0, y, w, h), "PBS",  lambda: self._start_auto(3), True); y += h+g
        self.b_solve  = B((x0, y, w, h), "SOLVE CBS", self._solve_or_cancel); y += h+g*2

        # ── Realtime ─────────────────────────────────────────────────
        self._y_realtime_label = y;                                                             y += 14+g
        self.b_auto_pibt  = B((x0, y, w, h), "PIBT", lambda: self._start_auto(4), True); y += h+g*2

        # ── Production ───────────────────────────────────────────────
        self._y_production_label = y;                                                           y += 14+g
        self.b_auto_astar = B((x0, y, w, h), "A* + Reservation", lambda: self._start_auto(0), True); y += h+g


        # ── Map / Data ───────────────────────────────────────────────
        self._y_map_label = y;                                                                   y += 14+g
        self.b_save  = B((x0,       y, hw, h), "Save Map", self._save_map)
        self.b_load  = B((x0+hw+4,  y, hw, h), "Load Map", self._load_map);                y += h+g
        self.b_new   = B((x0, y, w, h), "New Map...",     self._new_map);                  y += h+g
        self.b_clag  = B((x0, y, w, h), "Clear Agents",   self._clear_agents);             y += h+g
        self.b_call  = B((x0, y, w, h), "Clear All",      self._clear_all)

        self._btns = [
            # Toolbar
            self.b_obs, self.b_era, self.b_forbid, self.b_speed, self.b_viz,
            # Sidebar - Agent
            self.b_agnt, self.b_add, self.b_del, self.b_start, self.b_goal, self.b_rand,
            self.b_auto_astar, self.b_auto_cbs, self.b_auto_rhcr, self.b_auto_pbs, self.b_auto_pibt,
            self.b_solve,
            self.b_save, self.b_load,
            self.b_new, self.b_clag, self.b_call,
        ]
        self._refresh_btn_state()

    def _refresh_btn_state(self):
        self.b_obs.active      = (self.mode == MODE_OBS)
        self.b_era.active      = (self.mode == MODE_ERASE)
        self.b_forbid.active   = (self.mode == MODE_FORBID)
        self.b_speed.active    = (self.mode == MODE_SPEED)
        self.b_agnt.active     = (self.mode == MODE_AGNT)
        self.b_start.active    = (self.edit_sub == SUB_START)
        self.b_goal.active     = (self.edit_sub == SUB_GOAL)
        self.b_viz.active        = self._astar_viz
        self.b_auto_astar.active = self._cont_mode and self._cont_replan_mode == 0
        self.b_auto_cbs.active   = self._cont_mode and self._cont_replan_mode == 1
        self.b_auto_rhcr.active  = self._cont_mode and self._cont_replan_mode == 2
        self.b_auto_pbs.active   = self._cont_mode and self._cont_replan_mode == 3
        self.b_auto_pibt.active  = self._cont_mode and self._cont_replan_mode == 4

    def _reposition_btns(self):
        self._rebuild_btns()

    # ── Helpers ────────────────────────────────────────────────────────────────
    def _grid_area_w(self):
        return self.screen.get_width() - SIDEBAR_W

    def _screen_to_grid(self, sx, sy):
        c = int((sx - self.cam_x) / self.cell)
        r = int((sy - self.cam_y) / self.cell)
        if 0 <= c < self.grid.width and 0 <= r < self.grid.height:
            return c, r
        return None

    def _grid_to_screen(self, c, r):
        return (
            int(c * self.cell + self.cam_x),
            int(r * self.cell + self.cam_y),
        )

    def _check_collisions(self):
        """Detect agent-agent collisions each frame and update counters."""
        import math
        conflict_dist = 2.0 * self.agent_radius
        positions = {}

        if self._cont_mode and self._cont_replan_mode == 4:
            # PIBT mode: use current step positions
            positions = dict(self._pibt_cur)
        elif self._cont_mode and self._cont_agents:
            # Other auto modes: interpolated position
            for idx, ca in self._cont_agents.items():
                path = ca["path"]
                step = min(int(ca["local_t"]), len(path) - 1)
                positions[idx] = path[step]
        elif self.paths and self.sim_run:
            # Manual CBS playback
            for i, path in self.paths.items():
                step = min(self.sim_step, len(path) - 1)
                positions[i] = path[step]
        else:
            self._collision_active = set()
            self._collision_agents = set()
            return

        ids = list(positions.keys())
        now_colliding = set()
        now_agents    = set()

        for a in range(len(ids)):
            for b in range(a + 1, len(ids)):
                i, j = ids[a], ids[b]
                pi, pj = positions[i], positions[j]
                dist = math.sqrt((pi[0]-pj[0])**2 + (pi[1]-pj[1])**2)
                if dist < conflict_dist:
                    pair = frozenset((i, j))
                    now_colliding.add(pair)
                    now_agents.add(i)
                    now_agents.add(j)
                    # Count only NEW collisions (not already active)
                    if pair not in self._collision_active:
                        self._collision_count += 1
                        self._log("collision", a1=i, a2=j, dist=round(dist, 3),
                                  count=self._collision_count)
                        ts = f"{time.time():.1f}"
                        entry = f"#{self._collision_count:3d}  A{i+1} <-> A{j+1}"
                        self._collision_log.append(entry)
                        if len(self._collision_log) > self._collision_log_max:
                            self._collision_log.pop(0)

        self._collision_active = now_colliding
        self._collision_agents = now_agents

    def _record_arrival(self):
        """Call each time any agent completes a task. Updates throughput."""
        now = time.time()
        self._throughput_window.append(now)
        # Keep only last 60 seconds
        cutoff = now - 60.0
        self._throughput_window = [t for t in self._throughput_window if t > cutoff]
        # tasks/min = count in last 60s
        self._throughput = len(self._throughput_window)

    def _log(self, event, **data):
        elapsed = time.time() - self._sim_log_t0
        entry = {"t": round(elapsed, 3), "event": event}
        entry.update(data)
        self._sim_log.append(entry)

    def _flush_log(self):
        if not self._sim_log:
            return
        import json
        log_path = os.path.join(os.path.dirname(__file__), "sim_log.json")
        mode_names = {0: "A*", 1: "CBS", 2: "RHCR", 3: "PBS", 4: "PIBT"}
        header = {
            "mode": mode_names.get(self._cont_replan_mode, "manual"),
            "map_size": f"{self.grid.width}x{self.grid.height}",
            "resolution": self.grid.resolution,
            "agent_radius": self.agent_radius,
            "agent_count": len(self._cont_agents) if self._cont_agents else len(self.agents),
            "total_entries": len(self._sim_log),
        }
        try:
            with open(log_path, "w", encoding="utf-8") as f:
                json.dump({"info": header, "log": self._sim_log}, f, indent=1)
            self._show(f"Log saved: {len(self._sim_log)} entries")
        except Exception:
            pass

    def _toggle_astar_viz(self):
        if self._astar_viz:
            self._astar_viz = False
            self._astar_viz_agents = {}
        else:
            self._astar_viz = True
            self._astar_viz_agents = {}
            if not self._cont_mode:
                self._run_astar_viz()
        self._refresh_btn_state()

    def _run_astar_viz(self):
        valid = [(i, a) for i, a in enumerate(self.agents) if a.get("start") and a.get("goal")]
        if not valid:
            self._show("No agents with start+goal.")
            return
        self._astar_viz_agents = {}
        self._astar_viz_total = 0
        self._astar_viz = True

        def worker():
            from astar import astar
            for idx, agent in valid:
                self._astar_viz_agents[idx] = {}
                cells = self._astar_viz_agents[idx]
                def on_expand(c, r, t, total, _c=cells):
                    _c[(c, r)] = t
                    self._astar_viz_total = sum(len(v) for v in self._astar_viz_agents.values())
                astar(self.grid, agent["start"], agent["goal"],
                      time_per_cell=agent.get("tpc", 1),
                      on_expand=on_expand)

        threading.Thread(target=worker, daemon=True).start()

    def _time_scale(self):
        return BASE_SPEED / self.grid.resolution

    def _show(self, msg, secs=3):
        self.msg       = msg
        self.msg_ticks = secs * 60

    def _max_tpc(self):
        """Max tpc across all agents (≥1). H must be ≥ this."""
        if not self.agents:
            return 1
        return max(a.get("tpc", 1) for a in self.agents)

    # ── Mode / agent actions ───────────────────────────────────────────────────
    def _set_mode(self, m):
        self.mode     = m
        self.edit_sub = SUB_NONE
        self._refresh_btn_state()

    def _toggle_start(self):
        self.edit_sub = SUB_NONE if self.edit_sub == SUB_START else SUB_START
        self.mode     = MODE_AGNT
        self._refresh_btn_state()

    def _toggle_goal(self):
        self.edit_sub = SUB_NONE if self.edit_sub == SUB_GOAL else SUB_GOAL
        self.mode     = MODE_AGNT
        self._refresh_btn_state()

    def _adjust_agent_speed(self, ai, direction):
        """direction: +1 = faster (lower tpc), -1 = slower (higher tpc)"""
        cur_tpc = self.agents[ai].get("tpc", 1)
        new_tpc = cur_tpc - direction
        new_tpc = max(1, min(200, new_tpc))
        self.agents[ai]["tpc"] = new_tpc
        if ai in self._cont_agents:
            self._cont_agents[ai]["tpc"] = new_tpc
        new_ms = BASE_SPEED / new_tpc
        mt = max(2, self._max_tpc())
        if self._rhcr_h < mt:
            self._rhcr_h = mt
            self._rhcr_window = max(self._rhcr_window, self._rhcr_h)
            self._show(f"A{ai+1}: {new_ms:.3f}m/s  (H→{self._rhcr_h})")
        else:
            self._show(f"A{ai+1}: {new_ms:.3f}m/s")

    def _add_agent(self):
        if len(self.agents) >= MAX_AGENTS:
            self._show(f"Max {MAX_AGENTS} agents.")
            return
        self.agents.append({"start": None, "goal": None, "tpc": 1})
        self.sel      = len(self.agents) - 1
        self.mode     = MODE_AGNT
        self.edit_sub = SUB_START
        self._refresh_btn_state()
        self._show(f"Agent {self.sel+1} added - click grid to set start.")

    def _del_agent(self):
        if not self.agents:
            return
        self.agents.pop(self.sel)
        self.sel = max(0, self.sel - 1)
        self._reset_sim()

    def _add_or_remove_obstacle(self, c, r, placing):
        self.grid.set_obstacle(c, r, placing)
        self._invalidate_grid_cache()
        # Remove agent markers that overlap
        for a in self.agents:
            if a["start"] == (c, r) and placing:
                a["start"] = None
            if a["goal"] == (c, r) and placing:
                a["goal"] = None

    # ── Grid interaction ───────────────────────────────────────────────────────
    def _on_grid_click(self, sx, sy, btn):
        gp = self._screen_to_grid(sx, sy)
        if gp is None:
            return
        c, r = gp

        if self.mode == MODE_OBS:
            placing = btn == 1
            self._drag_painting = True
            self._drag_val      = placing
            self._box_start     = (c, r)
            self._box_cur       = (c, r)

        elif self.mode == MODE_ERASE:
            self._drag_painting = True
            self._drag_val      = False
            self._box_start     = (c, r)
            self._box_cur       = (c, r)

        elif self.mode == MODE_FORBID:
            self._drag_painting = True
            self._drag_val      = (btn == 1)
            self._box_start     = (c, r)
            self._box_cur       = (c, r)

        elif self.mode == MODE_SPEED:
            self._drag_painting = True
            self._drag_val      = (btn == 1)
            self._box_start     = (c, r)
            self._box_cur       = (c, r)

        elif self.mode == MODE_AGNT:
            if not self.agents:
                self._show("Add an agent first.")
                return
            agent = self.agents[self.sel]

            if self.edit_sub == SUB_START:
                if self.grid.is_obstacle(c, r) or (c, r) in self.grid.forbidden:
                    self._show("Cannot place start on blocked cell.")
                    return
                agent["start"] = (c, r)
                self.edit_sub  = SUB_GOAL
                self._refresh_btn_state()
                self._show("Start set – now click goal.")

            elif self.edit_sub == SUB_GOAL:
                if self.grid.is_obstacle(c, r) or (c, r) in self.grid.forbidden:
                    self._show("Cannot place goal on blocked cell.")
                    return
                agent["goal"]  = (c, r)
                self.edit_sub  = SUB_NONE
                self._refresh_btn_state()
                self._show(f"Agent {self.sel+1} start+goal set.")

            else:
                # Click to select agent by position
                for i, a in enumerate(self.agents):
                    if a["start"] == (c, r) or a["goal"] == (c, r):
                        self.sel = i
                        break

    def _on_grid_drag(self, sx, sy):
        if not self._drag_painting:
            return
        gp = self._screen_to_grid(sx, sy)
        if gp and self.mode in (MODE_OBS, MODE_ERASE, MODE_FORBID, MODE_SPEED):
            self._box_cur = gp

    def _apply_box(self):
        """Fill/erase the rectangle from _box_start to _box_cur."""
        if not self._box_start or not self._box_cur:
            return
        c0, r0 = self._box_start
        c1, r1 = self._box_cur
        for r in range(min(r0, r1), max(r0, r1) + 1):
            for c in range(min(c0, c1), max(c0, c1) + 1):
                if self.mode == MODE_FORBID:
                    if self._drag_val:
                        self.grid.forbidden.add((c, r))
                        self.grid.obstacles.discard((c, r))
                        self.grid.speed_zones.pop((c, r), None)
                    else:
                        self.grid.forbidden.discard((c, r))
                elif self.mode == MODE_SPEED:
                    if self._drag_val:
                        self.grid.speed_zones[(c, r)] = self._zone_speed_tpc
                        self.grid.obstacles.discard((c, r))
                        self.grid.forbidden.discard((c, r))
                    else:
                        self.grid.speed_zones.pop((c, r), None)
                else:
                    self._add_or_remove_obstacle(c, r, self._drag_val)
        self._box_start = None
        self._box_cur   = None
        self._invalidate_grid_cache()

    # ── CBS solve ─────────────────────────────────────────────────────────────
    def _solve(self):
        # Track which self.agents indices are valid so paths can be remapped
        self._valid_indices = [i for i, a in enumerate(self.agents) if a["start"] and a["goal"]]
        valid = [self.agents[i] for i in self._valid_indices]
        if not valid:
            self._show("No agents with start + goal defined.")
            return
        if self._solving:
            return
        self._reset_sim()
        self._solving     = True
        self._sol_result  = None
        self._sol_error   = ""
        self._stop_event.clear()
        self._cbs_progress.clear()
        self._solve_start = time.time()
        threading.Thread(target=self._worker, args=(valid, self.agent_radius), daemon=True).start()

    def _solve_or_cancel(self):
        if self._solving:
            self._stop_event.set()
            self._show("Solve cancelled.")
        else:
            self._solve()

    def _worker(self, valid_agents, agent_radius):
        tpc = [self.agents[i].get("tpc", 1)
               for i in self._valid_indices]
        result = solve_cbs(
            self.grid, valid_agents,
            max_time=400, agent_radius=agent_radius,
            stop_event=self._stop_event,
            progress=self._cbs_progress,
            max_nodes=self.max_nodes,
            time_per_cell=tpc,
        )
        terminated = self._cbs_progress.get('terminated', '')
        nodes      = self._cbs_progress.get('nodes', 0)
        if result is not None:
            self._sol_result = result
        elif terminated == 'no_solution':
            self._sol_error = f"[PROVEN] No solution exists. ({nodes} nodes)"
        elif terminated == 'node_limit':
            self._sol_error = f"[NODE LIMIT {self.max_nodes}] Too complex - raise limit or reduce agents/radius."
        elif terminated == 'cancelled':
            elapsed = time.time() - self._solve_start
            if elapsed >= self.solve_timeout - 0.5:
                self._sol_error = f"[TIMEOUT {self.solve_timeout:.0f}s] Search stopped - too complex."
            else:
                self._sol_error = "Cancelled."
        else:
            self._sol_error = "Terminated (unknown reason)."
        self._solving = False

    def _reset_sim(self):
        self.sim_run     = False
        self.sim_t       = 0.0
        self.sim_step    = 0
        self.paths       = {}
        self._sol_result = None
        self._collision_count  = 0
        self._collision_log.clear()
        self._collision_active = set()
        self._collision_agents = set()
        # Stop cont mode without clearing its state (let _toggle handle that)
        if not self._cont_mode:
            self._cont_agents = {}
            self._cont_total  = 0

    # ── Config persistence ────────────────────────────────────────────────────
    def _save_config(self, map_path=None):
        import json
        config_path = os.path.join(os.path.dirname(__file__), "config.json")
        # Load existing to preserve last_map if not overwriting
        cfg = {}
        if os.path.exists(config_path):
            try:
                with open(config_path, encoding="utf-8") as f:
                    cfg = json.load(f)
            except Exception:
                pass

        if map_path:
            cfg["last_map"] = os.path.abspath(map_path)
        cfg["cols"]          = self.grid.width
        cfg["rows"]          = self.grid.height
        cfg["cell"]          = self.cell
        cfg["agent_radius"]  = self.agent_radius
        cfg["sim_speed"]     = self.sim_speed
        cfg["solve_timeout"] = self.solve_timeout
        cfg["max_nodes"]     = self.max_nodes
        cfg["rhcr_window"]   = self._rhcr_window
        cfg["rhcr_h"]        = self._rhcr_h

        try:
            with open(config_path, "w", encoding="utf-8") as f:
                json.dump(cfg, f, indent=2)
        except Exception:
            pass

    # ── Map I/O ───────────────────────────────────────────────────────────────
    def _save_map(self):
        try:
            from tkinter import filedialog, Tk
            root = Tk(); root.withdraw()
            path = filedialog.asksaveasfilename(
                defaultextension=".yaml",
                filetypes=[("YAML files", "*.yaml"), ("All files", "*.*")],
                initialdir=os.path.join(os.path.dirname(__file__), "maps"),
            )
            root.destroy()
            if path:
                pgm = os.path.splitext(path)[0] + ".pgm"
                self.grid.save_pgm_yaml(pgm, path)
                self._save_config(path)
                self._show(f"Saved: {os.path.basename(path)}")
        except Exception as e:
            self._show(f"Save error: {e}")

    def _load_map(self):
        try:
            from tkinter import filedialog, Tk
            root = Tk(); root.withdraw()
            path = filedialog.askopenfilename(
                filetypes=[("YAML files", "*.yaml"), ("All files", "*.*")],
                initialdir=os.path.join(os.path.dirname(__file__), "maps"),
            )
            root.destroy()
            if path:
                grid, _ = Grid.from_pgm_yaml(path)
                self.grid = grid
                self._invalidate_grid_cache()
                self._save_config(path)
                self._reset_sim()
                self._clamp_cam()
                self._rebuild_btns()
                sw = self.grid.width  * self.cell + SIDEBAR_W
                sh = max(self.grid.height * self.cell, 520)
                info = pygame.display.Info()
                sw = min(sw, info.current_w - 40)
                sh = min(sh, info.current_h - 80)
                self.screen = pygame.display.set_mode((sw, sh), pygame.RESIZABLE)
                self._show(f"Loaded: {os.path.basename(path)}")
        except Exception as e:
            self._show(f"Load error: {e}")

    def _clear_agents(self):
        self.agents = []
        self.sel    = 0
        self._reset_sim()

    def _clear_all(self):
        self.grid   = Grid.empty(self.grid.width, self.grid.height)
        self.grid.forbidden.clear()
        self.grid.speed_zones.clear()
        self._invalidate_grid_cache()
        self.agents = []
        self.sel    = 0
        self._reset_sim()

    # ── Continuous / Auto mode ────────────────────────────────────────────────

    def _stop_cont_mode(self):
        self._log("sim_stop", total_arrivals=self._cont_total,
                  collisions=self._collision_count)
        self._flush_log()
        self._cont_mode     = False
        self._cont_agents   = {}
        self._safe_cache    = None
        self._cont_cbs_run  = False
        self._cont_cbs_pend = False
        self._cont_cbs_res  = None
        self._cont_progress = {}
        self._rhcr_global_t = 0.0
        self._rhcr_last_t   = 0.0
        self._rhcr_arrived.clear()
        self._collision_count  = 0
        self._collision_log.clear()
        self._collision_active = set()
        self._collision_agents = set()
        self._show("Auto mode stopped.")
        self._refresh_btn_state()

    def _start_auto(self, mode):
        """Start auto mode with the given replan mode (0=A*, 1=CBS, 2=RHCR).
        If the same mode is already active, stop it. Otherwise switch modes."""
        if self._cont_mode and self._cont_replan_mode == mode:
            self._stop_cont_mode()
        elif self._cont_mode:
            self._stop_cont_mode()
            self._cont_replan_mode = mode
            self._start_cont_mode()
            self._refresh_btn_state()
        else:
            self._cont_replan_mode = mode
            self._start_cont_mode()
            self._refresh_btn_state()

    def _start_cont_mode(self):
        import random
        from cbs import compute_safe_positions
        from astar import astar

        valid = [(i, a) for i, a in enumerate(self.agents) if a["start"] and a["goal"]]
        if not valid:
            self._show("No agents ready. Set start+goal or use 'Random Agents'.")
            return

        self._show("Building safe map...")
        self._safe_cache = compute_safe_positions(self.grid, self.agent_radius)
        safe = self._safe_cache

        self._cont_agents = {}
        self._cont_total  = 0

        for idx, a in valid:
            # Use current CBS position if available, else start
            if idx in self.paths:
                st = min(self.sim_step, len(self.paths[idx]) - 1)
                start = self.paths[idx][st]
            else:
                start = a["start"]

            goal = a["goal"]
            if start not in safe:
                start = min(safe, key=lambda p: abs(p[0]-start[0]) + abs(p[1]-start[1]))
            if goal not in safe:
                goal = min(safe, key=lambda p: abs(p[0]-goal[0]) + abs(p[1]-goal[1]))

            path = astar(self.grid, start, goal, safe_positions=safe)
            if path is None:
                path = [start]

            self._cont_agents[idx] = {
                "path":     path,
                "local_t":  0.0,
                "goal":     goal,
                "arrivals": 0,
                "tpc":      a.get("tpc", 1),
            }

        self._sim_log = []
        self._sim_log_t0 = time.time()
        self._log("sim_start", mode=self._cont_replan_mode,
                  map_size=f"{self.grid.width}x{self.grid.height}",
                  resolution=self.grid.resolution,
                  map_m=f"{self.grid.width*self.grid.resolution:.1f}x{self.grid.height*self.grid.resolution:.1f}",
                  agent_radius=self.agent_radius,
                  radius_m=round(self.agent_radius * self.grid.resolution, 3),
                  obstacles=len(self.grid.obstacles),
                  forbidden=len(self.grid.forbidden),
                  speed_zones=len(self.grid.speed_zones),
                  rhcr_w=self._rhcr_window,
                  rhcr_h=self._rhcr_h,
                  agents={str(idx): {"start": list(ca["path"][0]), "goal": list(ca["goal"]), "tpc": ca["tpc"],
                                     "speed_ms": round(BASE_SPEED / ca["tpc"], 3)}
                          for idx, ca in self._cont_agents.items()})
        self._cont_mode = True   # set BEFORE _reset_sim so cont_agents aren't cleared
        self._rhcr_arrived.clear()
        self._rhcr_global_t = 0.0
        if self._cont_replan_mode == 2:
            # Trigger first replan immediately so RHCR starts coordinated
            self._rhcr_last_t = -float(self._rhcr_h)
            self._rhcr_first_cbs = True  # don't move until first CBS result
        else:
            self._rhcr_last_t = 0.0
        if self._cont_replan_mode == 4:
            self._pibt_prev      = {idx: ca["path"][0] for idx, ca in self._cont_agents.items()}
            self._pibt_cur       = dict(self._pibt_prev)
            self._pibt_frac      = 0.0
            self._pibt_paths     = {}
            self._pibt_path_tick = 0
        self._reset_sim()
        mode_names = ["A*", "CBS", "RHCR", "PBS", "PIBT"]
        self._show(f"Auto({mode_names[self._cont_replan_mode]}): {len(self._cont_agents)} agents running.")

    def _pick_new_goal(self, idx):
        """Pick a spread-out random goal for agent idx, avoiding other agents/goals."""
        import random, math
        safe     = self._safe_cache or set()
        min_dist = self.agent_radius * 2 + 0.5

        # PIBT mode uses _pibt_cur for actual positions
        if self._cont_replan_mode == 4:
            cur = self._pibt_cur.get(idx, (0, 0))
            others = []
            for k in self._cont_agents:
                if k == idx:
                    continue
                others.append(self._pibt_cur.get(k, (0, 0)))
                others.append(self._cont_agents[k]["goal"])
        else:
            ca   = self._cont_agents[idx]
            path = ca["path"]
            cur  = path[min(int(ca["local_t"]), len(path) - 1)]
            others = []
            for k, v in self._cont_agents.items():
                if k == idx:
                    continue
                op = v["path"]
                os = min(int(v["local_t"]), len(op) - 1)
                others.append(op[os])
                others.append(v["goal"])

        cands = [
            p for p in safe
            if p != cur and
            all(math.sqrt((p[0]-o[0])**2 + (p[1]-o[1])**2) >= min_dist for o in others)
        ]
        if not cands:
            cands = [p for p in safe if p != cur]
        return random.choice(cands) if cands else None

    # Pre-computed body footprint offsets for the current agent_radius
    # (cached to avoid repeating sqrt every replan)
    _footprint_cache: tuple = (None, [])   # (radius, offsets)

    def _body_offsets(self):
        """Return list of (dc, dr) offsets within 2*agent_radius (cached)."""
        import math
        radius = self.agent_radius * 2
        cached_r, offsets = MAPFApp._footprint_cache
        if cached_r != radius:
            r_ceil = int(math.ceil(radius - 0.001))
            offsets = [
                (dc, dr)
                for dc in range(-r_ceil, r_ceil + 1)
                for dr in range(-r_ceil, r_ceil + 1)
                if math.sqrt(dc*dc + dr*dr) < radius
            ]
            MAPFApp._footprint_cache = (radius, offsets)
        return offsets

    _CONSTRAINT_HORIZON = 30   # only look ahead this many steps (speed vs accuracy)

    def _cont_replan_agent(self, idx):
        """Replan path for one agent to its CURRENT goal using soft constraints from others.
        Does NOT change the goal — goal assignment is the caller's responsibility."""
        from astar import astar

        safe = self._safe_cache
        if safe is None:
            return

        ca    = self._cont_agents[idx]
        path  = ca["path"]
        step  = min(int(ca["local_t"]), len(path) - 1)
        start = path[step]
        goal  = ca["goal"]

        offsets = self._body_offsets()
        horizon = self._CONSTRAINT_HORIZON
        vc = set()
        for oi, oa in self._cont_agents.items():
            if oi == idx:
                continue
            op  = oa["path"]
            ot  = min(int(oa["local_t"]), len(op) - 1)
            end = min(ot + horizon, len(op))      # limit lookahead
            for dt, pi in enumerate(range(ot, end)):
                c, r = op[pi]
                for dc, dr in offsets:
                    vc.add((c + dc, r + dr, dt + 1))

        t0 = time.time()
        new_path = astar(self.grid, start, goal, vc, set(),
                         max_time=200, safe_positions=safe)
        if new_path is None:
            new_path = astar(self.grid, start, goal, safe_positions=safe)
        if new_path is None:
            new_path = [start]
        self._stat_replan_ms  = (time.time() - t0) * 1000
        self._stat_replan_cnt += 1

        ca["path"]    = new_path
        ca["local_t"] = 0.0

    def _launch_astar_batch(self):
        """Run pending A* replans in a background thread."""
        if self._astar_running or not self._astar_pending:
            return
        batch = dict(self._astar_pending)
        self._astar_pending.clear()
        self._astar_inflight.update(batch.keys())
        self._astar_running = True

        def worker():
            t0 = time.time()
            results = {}
            for idx, goal in batch.items():
                if idx not in self._cont_agents:
                    continue
                ca    = self._cont_agents[idx]
                path  = ca["path"]
                step  = min(int(ca["local_t"]), len(path) - 1)
                start = path[step]
                safe  = self._safe_cache

                from astar import astar, _HAS_C, _astar_2d_c
                viz_cb = None
                if self._astar_viz:
                    self._astar_viz_agents[idx] = {}
                    _cells = self._astar_viz_agents[idx]
                    def viz_cb(c, r, t, total, _c=_cells):
                        _c[(c, r)] = t

                if _HAS_C and viz_cb is None:
                    new_path = _astar_2d_c(self.grid, start, goal, safe)
                else:
                    new_path = astar(self.grid, start, goal, safe_positions=safe, on_expand=viz_cb)
                if new_path is None:
                    new_path = [start]
                results[idx] = new_path

            elapsed = (time.time() - t0) * 1000
            self._astar_results  = results
            self._stat_replan_ms  = elapsed
            self._stat_replan_cnt += len(results)
            self._astar_running   = False

        threading.Thread(target=worker, daemon=True).start()

    def _launch_pbs_batch(self):
        """PBS per-arrival: replan each pending agent using ALL other agents'
        full paths as hard space-time constraints (unlike A* which uses a 30-step horizon)."""
        if self._astar_running or not self._astar_pending:
            return
        batch = dict(self._astar_pending)
        self._astar_pending.clear()
        self._astar_inflight.update(batch.keys())
        self._astar_running = True

        def worker():
            import math
            t0 = time.time()
            results = {}
            offsets = self._body_offsets()
            safe    = self._safe_cache

            for idx, goal in batch.items():
                if idx not in self._cont_agents:
                    continue
                ca    = self._cont_agents[idx]
                path  = ca["path"]
                step  = min(int(ca["local_t"]), len(path) - 1)
                start = path[step]

                vc = set()
                ec = set()
                for oi, oa in self._cont_agents.items():
                    if oi == idx:
                        continue
                    op  = oa["path"]
                    ot  = min(int(oa["local_t"]), len(op) - 1)
                    frozen = (ot >= len(op) - 1)

                    if frozen:
                        c, r = op[ot]
                        for future_dt in range(300):
                            for dc, dr in offsets:
                                vc.add((c + dc, r + dr, future_dt + 1))
                    else:
                        # Use FULL remaining path (not limited by _CONSTRAINT_HORIZON)
                        for dt, pi in enumerate(range(ot, len(op))):
                            c, r = op[pi]
                            for dc, dr in offsets:
                                vc.add((c + dc, r + dr, dt + 1))
                        # After path ends: freeze at goal
                        remaining = len(op) - ot
                        c, r = op[-1]
                        for future_dt in range(remaining, remaining + 300):
                            for dc, dr in offsets:
                                vc.add((c + dc, r + dr, future_dt + 1))
                        # Swap constraints
                        for dt in range(1, len(op) - ot):
                            pi = ot + dt
                            pc, pr = op[pi]
                            ppc, ppr = op[pi - 1]
                            ec.add((pc, pr, ppc, ppr, dt - 1))

                from astar import astar
                new_path = astar(self.grid, start, goal, vc, ec,
                                 max_time=300, safe_positions=safe)
                if new_path is None:
                    new_path = astar(self.grid, start, goal,
                                     safe_positions=safe)
                if new_path is None:
                    new_path = [start]
                results[idx] = new_path

            elapsed = (time.time() - t0) * 1000
            self._astar_results  = results
            self._stat_replan_ms  = elapsed
            self._stat_replan_cnt += len(results)
            self._astar_running   = False

        threading.Thread(target=worker, daemon=True).start()

    def _cont_replan_pbs_all(self):
        """Launch PBS for all cont agents in background (round-trip style)."""
        if self._cont_cbs_run:
            return
        from pbs import solve_pbs

        agents_snap = []
        indices     = []
        for idx, ca in self._cont_agents.items():
            path  = ca["path"]
            step  = min(int(ca["local_t"]), len(path) - 1)
            agents_snap.append({"start": path[step], "goal": ca["goal"]})
            indices.append(idx)

        # Priority order: by _pbs_priority list (arrival count ascending = more equal)
        priority_order = list(range(len(agents_snap)))
        if self._pbs_priority:
            # Sort by arrival count ascending → fewer arrivals = higher priority (fairness)
            arrivals = [self._cont_agents[indices[i]].get("arrivals", 0)
                        for i in priority_order]
            priority_order.sort(key=lambda i: arrivals[i])

        self._stop_event.clear()
        self._cont_work_gen += 1
        my_gen = self._cont_work_gen
        self._cont_cbs_run = True
        self._cont_progress.clear()

        def worker():
            result = solve_pbs(
                self.grid, agents_snap,
                max_time=300, agent_radius=self.agent_radius,
                priority_order=priority_order,
                stop_event=self._stop_event,
                progress=self._cont_progress,
            )
            self._cont_cbs_res = (result, list(indices), my_gen)
            self._cont_cbs_run = False

        threading.Thread(target=worker, daemon=True).start()

    def _cont_replan_pbs_fallback(self, indices):
        """PBS fallback when CBS fails. Runs in background thread."""
        if self._cont_cbs_run:
            return
        from pbs import solve_pbs
        safe = self._safe_cache
        agents_snap = []
        for idx in indices:
            ca = self._cont_agents[idx]
            path = ca["path"]
            step = min(int(ca["local_t"]), len(path) - 1)
            start = path[step]
            agents_snap.append({"start": start, "goal": ca["goal"]})

        self._stop_event.clear()
        self._cont_work_gen += 1
        my_gen = self._cont_work_gen
        self._cont_cbs_run = True
        self._cont_cbs_snap = {idx: self._cont_agents[idx]["local_t"] for idx in indices}
        self._cont_cbs_frac = {idx: self._cont_agents[idx]["local_t"] - int(self._cont_agents[idx]["local_t"]) for idx in indices}
        self._cont_progress.clear()

        def worker():
            result = solve_pbs(
                self.grid, agents_snap,
                max_time=300, agent_radius=self.agent_radius,
                stop_event=self._stop_event,
                progress=self._cont_progress,
            )
            self._cont_cbs_res = (result, list(indices), my_gen)
            self._cont_cbs_run = False

        threading.Thread(target=worker, daemon=True).start()

    def _cont_replan_cbs_all(self):
        """Launch CBS for all cont agents in background. Agents keep moving on old paths.
        Records each agent's local_t at snapshot time so the result can be
        offset-compensated on arrival (skip steps already traveled during CBS runtime)."""
        if self._cont_cbs_run:
            self._cont_cbs_pend = True
            return

        from cbs import solve_cbs
        safe = self._safe_cache

        # Snapshot positions and fractional local_t for smooth path handoff
        agents_snap  = []
        indices      = []
        snap_local_t = {}
        snap_frac    = {}   # {idx: fractional part of local_t} for visual continuity
        for idx, ca in self._cont_agents.items():
            path  = ca["path"]
            lt    = ca["local_t"]
            step  = min(int(lt), len(path) - 1)
            start = path[step]
            goal  = ca["goal"]
            agents_snap.append({"start": start, "goal": goal})
            indices.append(idx)
            snap_local_t[idx] = lt
            snap_frac[idx]    = lt - int(lt)   # e.g. 4.97 → 0.97

        self._log("cbs_launch",
                  agents={str(idx): {"start": list(agents_snap[j]["start"]),
                                     "goal": list(agents_snap[j]["goal"]),
                                     "tpc": self._cont_agents[idx].get("tpc", 1)}
                          for j, idx in enumerate(indices)},
                  rhcr_w=self._rhcr_window if self._cont_replan_mode == 2 else None)

        self._stop_event.clear()
        self._cont_work_gen += 1
        my_gen = self._cont_work_gen
        self._cont_cbs_run  = True
        self._cont_cbs_snap = snap_local_t
        self._cont_cbs_frac = snap_frac
        self._cont_progress.clear()

        if self._cont_replan_mode == 2:
            rhcr_horizon = self._rhcr_window + self._rhcr_consec_fail * self._rhcr_window
            rhcr_horizon = min(rhcr_horizon, 300)
        else:
            rhcr_horizon = None
        tpc_snap = [self._cont_agents[idx].get("tpc", 1)
                    for idx in indices]

        def worker():
            result = solve_cbs(
                self.grid, agents_snap,
                max_time=300, agent_radius=self.agent_radius,
                stop_event=self._stop_event,
                max_nodes=self.max_nodes,
                progress=self._cont_progress,
                horizon=rhcr_horizon,
                time_per_cell=tpc_snap,
            )
            self._cont_cbs_res = (result, list(indices), my_gen)
            self._cont_cbs_run = False

        threading.Thread(target=worker, daemon=True).start()

    def _random_agents(self):
        import random
        from tkinter import simpledialog, Tk

        root = Tk(); root.withdraw()
        n = simpledialog.askinteger(
            "Random Agents", f"How many agents? (1-{MAX_AGENTS})",
            minvalue=1, maxvalue=MAX_AGENTS,
            initialvalue=min(10, MAX_AGENTS - len(self.agents))
        )
        if not n:
            root.destroy()
            return

        tpc_input = simpledialog.askstring(
            "Speed (m/s)",
            f"숫자(0.01~2.00 m/s): 전체 동일 속도\n'r' 또는 빈칸: 전체 랜덤",
            initialvalue="1.00"
        )
        root.destroy()

        tpc_input = (tpc_input or "").strip().lower()
        if tpc_input in ("r", ""):
            tpc_mode = "random"
        else:
            try:
                ms_val = max(0.01, min(2.0, float(tpc_input)))
                tpc_fixed = max(1, round(BASE_SPEED / ms_val))
                tpc_mode = "fixed"
            except (ValueError, ZeroDivisionError):
                tpc_mode = "random"

        from cbs import compute_safe_positions
        safe = list(compute_safe_positions(self.grid, self.agent_radius))

        # Remove positions already used by existing agents
        used = set()
        for a in self.agents:
            if a["start"]: used.add(a["start"])
            if a["goal"]:  used.add(a["goal"])
        available = [p for p in safe if p not in used]

        # Sample 2n positions where every pair is >= 2*radius apart
        # (prevents t=0 conflicts and goal conflicts)
        chosen = self._spread_sample(available, n * 2)
        if len(chosen) < 2:
            self._show("Not enough spread-out safe positions. Try smaller radius or larger map.")
            return
        # Round down to even count
        actual = len(chosen) // 2
        if actual < n:
            self._show(f"Only space for {actual} spread agents (radius={self.agent_radius}).")
            n = actual
        chosen  = chosen[:n * 2]
        starts, goals = chosen[:n], chosen[n:]

        added = 0
        for i in range(n):
            if len(self.agents) >= MAX_AGENTS:
                break
            if tpc_mode == "fixed":
                spd_tpc = tpc_fixed
            else:
                rand_ms = random.uniform(0.01, 2.0)
                spd_tpc = max(1, round(BASE_SPEED / rand_ms))
            self.agents.append({"start": starts[i], "goal": goals[i], "tpc": spd_tpc})
            added += 1

        self.sel = max(0, len(self.agents) - 1)
        self._reset_sim()
        self._show(f"Added {added} agents — no position overlaps (total: {len(self.agents)}).")

    def _new_map(self):
        try:
            import tkinter as tk
            from tkinter import simpledialog

            root = tk.Tk()
            root.withdraw()

            dlg = tk.Toplevel(root)
            dlg.title("New Map")
            dlg.resizable(False, False)
            dlg.grab_set()

            fields = {}
            res_def = self.grid.resolution
            w_m_def = round(self.grid.width * res_def, 2)
            h_m_def = round(self.grid.height * res_def, 2)
            defaults = [
                ("Resolution (m/cell)", res_def),
                ("Width  (m)",          w_m_def),
                ("Height (m)",          h_m_def),
            ]
            for row_i, (label, default) in enumerate(defaults):
                tk.Label(dlg, text=label, anchor="w", width=18).grid(
                    row=row_i, column=0, padx=10, pady=6, sticky="w")
                var = tk.StringVar(value=str(default))
                tk.Entry(dlg, textvariable=var, width=8).grid(
                    row=row_i, column=1, padx=10, pady=6)
                fields[label] = var

            # Live preview label
            preview_var = tk.StringVar()
            tk.Label(dlg, textvariable=preview_var, fg="gray").grid(
                row=3, column=0, columnspan=2, padx=10)

            def update_preview(*_):
                try:
                    r = float(fields["Resolution (m/cell)"].get())
                    wm = float(fields["Width  (m)"].get())
                    hm = float(fields["Height (m)"].get())
                    preview_var.set(f"{int(wm/r)} x {int(hm/r)} cells")
                except (ValueError, ZeroDivisionError):
                    preview_var.set("")

            for v in fields.values():
                v.trace_add("write", update_preview)
            update_preview()

            result = {}

            def ok():
                try:
                    res = max(0.01, min(10.0, float(fields["Resolution (m/cell)"].get())))
                    w_m = max(res * 4, min(100.0, float(fields["Width  (m)"].get())))
                    h_m = max(res * 4, min(100.0, float(fields["Height (m)"].get())))
                    result["cols"] = max(4, int(w_m / res))
                    result["rows"] = max(4, int(h_m / res))
                    result["resolution"] = res
                    dlg.destroy()
                except (ValueError, ZeroDivisionError):
                    pass

            def cancel():
                dlg.destroy()

            tk.Button(dlg, text="OK",     command=ok,     width=8).grid(row=4, column=0, pady=10)
            tk.Button(dlg, text="Cancel", command=cancel, width=8).grid(row=4, column=1, pady=10)
            dlg.bind("<Return>", lambda _: ok())
            dlg.bind("<Escape>", lambda _: cancel())

            root.wait_window(dlg)
            root.destroy()

            if result:
                self.grid   = Grid.empty(result["cols"], result["rows"])
                self.grid.resolution = result["resolution"]
                self.agents = []
                self.sel    = 0
                self.cam_x  = 0
                self.cam_y  = 0
                self._reset_sim()
                self._rebuild_btns()
                sw = self.grid.width  * self.cell + SIDEBAR_W
                sh = max(self.grid.height * self.cell, 520)
                info = pygame.display.Info()
                sw = min(sw, info.current_w - 40)
                sh = min(sh, info.current_h - 80)
                self.screen = pygame.display.set_mode((sw, sh), pygame.RESIZABLE)
                w_m = result['cols'] * result['resolution']
                h_m = result['rows'] * result['resolution']
                self._show(f"New map: {result['cols']}x{result['rows']} ({w_m:.1f}x{h_m:.1f}m, {result['resolution']}m/cell)")
        except Exception as e:
            self._show(f"Error: {e}")

    # ── Camera ────────────────────────────────────────────────────────────────
    def _spread_sample(self, positions, n, min_dist=None):
        """Sample up to n positions where every pair is >= min_dist apart.
        Defaults to 2*agent_radius + 0.5 so agents don't conflict at t=0."""
        import math, random
        if min_dist is None:
            min_dist = self.agent_radius * 2 + 0.5
        pool   = list(positions)
        random.shuffle(pool)
        result = []
        for pos in pool:
            if len(result) >= n:
                break
            if all(math.sqrt((pos[0]-s[0])**2 + (pos[1]-s[1])**2) >= min_dist
                   for s in result):
                result.append(pos)
        return result

    def _clamp_cam(self):
        ga_w = self._grid_area_w()
        ga_h = self.screen.get_height()
        map_w = self.grid.width  * self.cell
        map_h = self.grid.height * self.cell
        # Allow panning such that grid stays somewhat visible
        self.cam_x = max(-(map_w - 40), min(ga_w - 40, self.cam_x))
        self.cam_y = max(-(map_h - 40), min(ga_h - 40, self.cam_y))

    # ── Update ────────────────────────────────────────────────────────────────
    def update(self, dt):
        # Auto-timeout while solving
        if self._solving:
            elapsed = time.time() - self._solve_start
            if elapsed > self.solve_timeout:
                self._stop_event.set()

        # Pick up solve result from worker thread — remap to actual self.agents indices
        if self._sol_result is not None and not self.paths:
            for j, path in self._sol_result.items():
                self.paths[self._valid_indices[j]] = path
            self._sol_result = None
            self.sim_run     = True
            self.sim_t       = 0.0
            self.sim_step    = 0
            self._show("Solution found! Playing...")

        if self._sol_error:
            self._show(self._sol_error, secs=5)
            self._sol_error = ""

        if self.sim_run and self.paths:
            max_steps = max(len(p) for p in self.paths.values())
            self.sim_t    += dt * self.sim_speed * self._time_scale()
            self.sim_step  = int(self.sim_t)
            if self.sim_step >= max_steps:
                self.sim_step = max_steps - 1
                self.sim_run  = False

        # ── Continuous mode tick ──────────────────────────────────────────────
        if self._cont_mode and self._cont_agents:
            # Pick up A* batch results from worker
            if self._astar_results:
                for idx, new_path in self._astar_results.items():
                    if idx in self._cont_agents:
                        self._cont_agents[idx]["path"]    = new_path
                        self._cont_agents[idx]["local_t"] = 0.0
                        self._cont_agents[idx]["goal"]    = new_path[-1]
                        self._log("replan_astar", agent=idx, path_len=len(new_path),
                                  goal=list(new_path[-1]))
                    self._astar_inflight.discard(idx)
                self._astar_results = {}
                if self._astar_pending:
                    self._launch_astar_batch()

            # Pick up CBS/PBS result from worker (ignore stale results)
            if self._cont_cbs_res is not None:
                result, indices, res_gen = self._cont_cbs_res
                self._cont_cbs_res = None
                if res_gen != self._cont_work_gen:
                    pass  # stale result from previous mode — discard silently
                elif result:
                    frac = getattr(self, '_cont_cbs_frac', {})
                    for j, idx in enumerate(indices):
                        if idx not in self._cont_agents or j not in result:
                            continue
                        path = result[j]
                        if self._cont_replan_mode == 2:
                            path = path[:self._rhcr_window + 1]
                        self._cont_agents[idx]["path"] = path
                        self._cont_agents[idx]["local_t"] = frac.get(idx, 0.0)
                        self._rhcr_arrived.discard(idx)
                    self._rhcr_consec_fail = 0
                    self._rhcr_first_cbs = False
                    self._log("replan_cbs", status="ok",
                              nodes=self._cont_progress.get('nodes', 0),
                              agents=len(indices))
                    if self._cont_cbs_pend:
                        self._cont_cbs_pend = False
                else:
                    reason = self._cont_progress.get('terminated', 'unknown')
                    nodes  = self._cont_progress.get('nodes', 0)
                    safe_n = self._cont_progress.get('safe_count', '?')
                    self._log("replan_cbs", status="fail", reason=reason,
                              nodes=nodes, safe_cells=str(safe_n),
                              failed_agent=self._cont_progress.get('failed_agent',
                                           self._cont_progress.get('last_fail_agent', '?')),
                              failed_start=self._cont_progress.get('failed_start',
                                            self._cont_progress.get('last_fail_start', '?')),
                              failed_goal=self._cont_progress.get('failed_goal',
                                           self._cont_progress.get('last_fail_goal', '?')),
                              fail_v_constraints=self._cont_progress.get('last_fail_constraints_v', '?'),
                              fail_e_constraints=self._cont_progress.get('last_fail_constraints_e', '?'),
                              skipped_dupes=self._cont_progress.get('skipped_dupes', 0),
                              seen_combos=self._cont_progress.get('seen_combos', 0),
                              last_conflict=self._cont_progress.get('last_conflict', None))
                    if self._cont_replan_mode == 3:
                        self._show(
                            f"PBS failed [{reason}] safe_cells={safe_n} "
                            f"— try [z] to reduce radius", secs=6)
                    else:
                        self._rhcr_consec_fail += 1
                        self._rhcr_first_cbs = False
                        self._show(f"CBS failed — trying PBS fallback x{self._rhcr_consec_fail}", secs=3)
                        self._log("replan_fallback", reason="cbs_fail_pbs_retry",
                                  consec_fail=self._rhcr_consec_fail)
                        self._cont_replan_pbs_fallback(indices)
                    if self._cont_cbs_pend:
                        self._cont_cbs_pend = False

            import random, math
            min_dist = self.agent_radius * 2 + 0.5
            safe     = self._safe_cache or set()

            if self._cont_replan_mode == 1:
                # ── CBS round-trip: freeze during CBS ────────────────────
                if self._cont_cbs_run:
                    pass  # wait for CBS result
                else:
                    for idx, ca in self._cont_agents.items():
                        path = ca["path"]
                        at_goal = (int(ca["local_t"]) >= len(path) - 1
                                   and path[-1] == ca["goal"])
                        if not at_goal:
                            ca["local_t"] += dt * self.sim_speed * self._time_scale()

                    all_at_goal = all(
                        int(ca["local_t"]) >= len(ca["path"]) - 1
                        and ca["path"][-1] == ca["goal"]
                        for ca in self._cont_agents.values()
                    )
                    if all_at_goal:
                        self._log("round_complete", total=self._cont_total)
                        cur_positions = [ca["path"][-1] for ca in self._cont_agents.values()]
                        new_goals = self._spread_sample(
                            [p for p in safe if p not in cur_positions],
                            len(self._cont_agents)
                        )
                        for i, (idx, ca) in enumerate(self._cont_agents.items()):
                            ca["arrivals"] += 1
                            self._cont_total += 1
                            self._record_arrival()
                            self._log("arrival", agent=idx, pos=list(ca["path"][-1]), total=ca["arrivals"])
                            if i < len(new_goals):
                                ca["goal"] = new_goals[i]
                                self._log("new_goal", agent=idx, goal=list(new_goals[i]))
                        self._cont_replan_cbs_all()

            elif self._cont_replan_mode == 2:
                # ── RHCR: freeze during CBS to guarantee no collisions ────
                if self._cont_cbs_run or self._rhcr_first_cbs:
                    if self._rhcr_first_cbs:
                        self._show("RHCR: waiting for first CBS...", secs=1)
                    else:
                        self._show("RHCR: CBS computing...", secs=1)
                else:
                    self._rhcr_global_t += dt * self.sim_speed * self._time_scale()

                    # Advance agents; clamp at end of path (already trimmed to W)
                    for idx, ca in self._cont_agents.items():
                        path = ca["path"]
                        if int(ca["local_t"]) < len(path) - 1:
                            ca["local_t"] += dt * self.sim_speed * self._time_scale()
                        else:
                            ca["local_t"] = float(len(path) - 1)

                # Arrival detection (one-time per goal, guarded by _rhcr_arrived)
                for idx, ca in self._cont_agents.items():
                    if idx in self._rhcr_arrived:
                        continue
                    path = ca["path"]
                    if (int(ca["local_t"]) >= len(path) - 1
                            and path[-1] == ca["goal"]):
                        self._rhcr_arrived.add(idx)
                        ca["arrivals"] += 1
                        self._cont_total += 1
                        self._record_arrival()
                        self._log("arrival", agent=idx, pos=list(path[-1]), total=ca["arrivals"])
                        new_goal = self._pick_new_goal(idx)
                        if new_goal:
                            ca["goal"] = new_goal
                            self._log("new_goal", agent=idx, goal=list(new_goal))

                # H steps elapsed → launch CBS for next W-step window
                if self._rhcr_global_t - self._rhcr_last_t >= self._rhcr_h:
                    if not self._cont_cbs_run:
                        self._rhcr_last_t = self._rhcr_global_t
                        self._cont_replan_cbs_all()

            elif self._cont_replan_mode == 3:
                # ── PBS round-trip: ALL arrive → new goals → PBS replan ───────
                if not self._cont_cbs_run:
                    for idx, ca in self._cont_agents.items():
                        path = ca["path"]
                        at_goal = (int(ca["local_t"]) >= len(path) - 1
                                   and path[-1] == ca["goal"])
                        if not at_goal:
                            ca["local_t"] += dt * self.sim_speed * self._time_scale()

                    all_at_goal = all(
                        int(ca["local_t"]) >= len(ca["path"]) - 1
                        and ca["path"][-1] == ca["goal"]
                        for ca in self._cont_agents.values()
                    )
                    if all_at_goal:
                        cur_positions = [ca["path"][-1] for ca in self._cont_agents.values()]
                        new_goals = self._spread_sample(
                            [p for p in safe if p not in cur_positions],
                            len(self._cont_agents)
                        )
                        for i, (idx, ca) in enumerate(self._cont_agents.items()):
                            ca["arrivals"] += 1
                            self._cont_total += 1
                            self._record_arrival()
                            if i < len(new_goals):
                                ca["goal"] = new_goals[i]
                        self._cont_replan_pbs_all()

            elif self._cont_replan_mode == 4:
                # ── PIBT: one cell per step, priority inheritance ─────────────
                from pibt import pibt_step, manhattan as mb_manhattan
                self._pibt_frac += dt * self.sim_speed * self._time_scale()
                if self._pibt_frac >= 1.0:
                    self._pibt_frac -= 1.0
                    # Arrival check
                    for idx, pos in self._pibt_cur.items():
                        ca = self._cont_agents[idx]
                        if pos == ca["goal"]:
                            ca["arrivals"] += 1
                            self._cont_total += 1
                            self._record_arrival()
                            new_goal = self._pick_new_goal(idx)
                            if new_goal:
                                ca["goal"] = new_goal
                    # Track stuck counts (didn't move last step)
                    for sidx in self._cont_agents:
                        prev_p = self._pibt_prev.get(sidx)
                        cur_p  = self._pibt_cur.get(sidx, prev_p)
                        ca_s   = self._cont_agents[sidx]
                        if cur_p == prev_p:
                            ca_s["pibt_stuck"] = ca_s.get("pibt_stuck", 0) + 1
                        else:
                            ca_s["pibt_stuck"] = 0

                    # Advance: cur → prev, compute next cur
                    self._pibt_prev = dict(self._pibt_cur)
                    goals = {idx: self._cont_agents[idx]["goal"]
                             for idx in self._cont_agents}
                    # Priority: most stuck first → breaks deadlocks
                    priority_order = sorted(
                        self._cont_agents.keys(),
                        key=lambda i: (
                            -self._cont_agents[i].get("pibt_stuck", 0),
                             self._cont_agents[i].get("arrivals", 0)
                        )
                    )
                    # Store rank for visualization (0 = highest priority)
                    self._pibt_rank = {idx: rank for rank, idx in enumerate(priority_order)}
                    # Compute BFS distances for any new goals (cache by goal)
                    from pibt import bfs_dist as _bfs_dist
                    _safe = self._safe_cache or set()
                    for _idx, _g in goals.items():
                        if _g not in self._pibt_dist_cache:
                            self._pibt_dist_cache[_g] = _bfs_dist(self.grid, _g, _safe)

                    self._pibt_cur = pibt_step(
                        self.grid, self._pibt_prev, goals,
                        priority_order,
                        _safe,
                        self.agent_radius,
                        distances=self._pibt_dist_cache,
                    )

                    # Force-escape agents stuck ≥ 8 steps (deadlock break)
                    import math as _math
                    _conf = 2.0 * self.agent_radius
                    _safe = self._safe_cache or set()
                    for sidx, ca2 in self._cont_agents.items():
                        if ca2.get("pibt_stuck", 0) < 8:
                            continue
                        cur2 = self._pibt_prev.get(sidx)
                        if cur2 is None:
                            continue
                        others2 = {k: v for k, v in self._pibt_cur.items() if k != sidx}
                        free2 = [p for p in self.grid.neighbors4(*cur2)
                                 if p in _safe and
                                 all(_math.sqrt((p[0]-q[0])**2+(p[1]-q[1])**2) >= _conf
                                     for q in others2.values())]
                        if free2:
                            import random as _rnd
                            self._pibt_cur[sidx] = _rnd.choice(free2)
                            ca2["pibt_stuck"] = 0

            else:
                # ── A* mode: arrived OR stuck → assign new goal → replan ──────
                for idx, ca in self._cont_agents.items():
                    path = ca["path"]
                    at_goal = (int(ca["local_t"]) >= len(path) - 1
                               and path[-1] == ca["goal"])
                    if not at_goal:
                        ca["local_t"] += dt * self.sim_speed * self._time_scale() / max(1, ca.get("tpc", 1))

                for idx, ca in self._cont_agents.items():
                    if idx in self._astar_pending or idx in self._astar_inflight:
                        continue
                    path = ca["path"]
                    at_end = int(ca["local_t"]) >= len(path) - 1
                    if not at_end:
                        continue

                    arrived = path[-1] == ca["goal"]
                    if arrived:
                        ca["arrivals"] += 1
                        self._cont_total += 1
                        self._record_arrival()
                        self._log("arrival", agent=idx, pos=list(path[-1]), total=ca["arrivals"])

                    cur = path[-1]
                    others = []
                    for k, v in self._cont_agents.items():
                        if k == idx: continue
                        op = v["path"]
                        os = min(int(v["local_t"]), len(op)-1)
                        others.append(op[os])
                        others.append(v["goal"])
                    cands = [
                        p for p in safe
                        if p != cur and
                        all(math.sqrt((p[0]-o[0])**2+(p[1]-o[1])**2) >= min_dist
                            for o in others)
                    ]
                    if not cands:
                        cands = [p for p in safe if p != cur]
                    if cands:
                        ca["goal"] = random.choice(cands)
                    else:
                        continue

                    self._log("new_goal", agent=idx, goal=list(ca["goal"]))
                    self._astar_pending[idx] = ca["goal"]

                self._launch_astar_batch()

        if self.msg_ticks > 0:
            self.msg_ticks -= 1

        # Collision detection (skip during CBS freeze — robots not actually moving)
        if not (self._cont_mode and self._cont_cbs_run and self._cont_replan_mode in (1, 2)):
            self._check_collisions()

        # FPS & replans/sec tracking
        self._stat_fps      = self.clock.get_fps()
        self._stat_frame_ms = dt * 1000
        self._stat_replan_t += dt
        if self._stat_replan_t >= 1.0:
            self._stat_replan_ps  = self._stat_replan_cnt / self._stat_replan_t
            self._stat_replan_cnt = 0
            self._stat_replan_t   = 0.0

    # ── Draw ──────────────────────────────────────────────────────────────────
    def draw(self):
        self.screen.fill(BG)
        self._draw_grid()
        # Top toolbar background
        ga_w = self._grid_area_w()
        pygame.draw.rect(self.screen, SIDEBAR_BG, (0, 0, ga_w, self.TOOLBAR_H))
        pygame.draw.line(self.screen, SIDEBAR_EDGE, (0, self.TOOLBAR_H), (ga_w, self.TOOLBAR_H), 1)

        # Toolbar group labels and separators
        groups = [
            (self.b_obs.rect.left, self.b_era.rect.right, "Draw", (140, 160, 200)),
            (self.b_viz.rect.left, self.b_viz.rect.right, "View", (200, 180, 140)),
        ]
        for gx0, gx1, glabel, gcol in groups:
            pygame.draw.rect(self.screen, (35, 35, 45), (gx0 - 2, 1, gx1 - gx0 + 4, self.TOOLBAR_H - 2), border_radius=4)
            pygame.draw.rect(self.screen, (*gcol, 80), (gx0 - 2, 1, gx1 - gx0 + 4, self.TOOLBAR_H - 2), 1, border_radius=4)
            lbl = self.font_s.render(glabel, True, gcol)
            mid = (gx0 + gx1) // 2 - lbl.get_width() // 2
            self.screen.blit(lbl, (mid, self.TOOLBAR_H - 12))
        self._draw_astar_viz()
        self._draw_box_preview()
        if self._solving and self._show_progress:
            self._draw_cbs_progress()
        if self._cont_mode and self._cont_cbs_run and self._show_progress:
            self._draw_cbs_progress(progress=self._cont_progress)
        if self._cont_mode and self._cont_replan_mode == 4:
            self._draw_pibt_agents()
        elif self._cont_mode:
            self._draw_cont_agents()
        else:
            self._draw_paths()
            self._draw_agents()
        self._draw_sidebar()
        self._draw_stat_bar()
        self._draw_collision_log()
        if self.msg and self.msg_ticks > 0:
            self._draw_msg()
        self._draw_tooltip()
        pygame.display.flip()

    def _draw_cont_agents(self):
        cell   = self.cell
        radius = max(4, int(self.agent_radius * cell))

        for idx, ca in self._cont_agents.items():
            color  = agent_color(idx)
            dim    = tuple(max(0, v - 100) for v in color)
            path   = ca["path"]
            local_t = ca["local_t"]
            step   = min(int(local_t), len(path) - 1)
            frac   = local_t - int(local_t)

            thick_w = max(2, cell // 4)

            # RHCR: thick up to W steps (conflict-guaranteed), thin beyond
            # CBS round-trip: all thick. A*: all thin.
            if self._cont_replan_mode == 2:
                guaranteed_end = self._rhcr_window   # fixed at CBS-planned W steps
            elif self._cont_replan_mode == 1:
                guaranteed_end = len(path)
            else:
                guaranteed_end = len(path)  # A*: show full path, thin

            draw_thick_end = min(guaranteed_end, len(path) - 1)

            # Thin line: full remaining path (all modes)
            for j in range(step, len(path) - 1):
                x0 = int(path[j][0]   * cell + cell/2 + self.cam_x)
                y0 = int(path[j][1]   * cell + cell/2 + self.cam_y)
                x1 = int(path[j+1][0] * cell + cell/2 + self.cam_x)
                y1 = int(path[j+1][1] * cell + cell/2 + self.cam_y)
                pygame.draw.line(self.screen, dim, (x0, y0), (x1, y1), 1)

            # Thick line: CBS/RHCR guaranteed range on top
            if self._cont_replan_mode in (1, 2):
                for j in range(step, draw_thick_end):
                    x0 = int(path[j][0]   * cell + cell/2 + self.cam_x)
                    y0 = int(path[j][1]   * cell + cell/2 + self.cam_y)
                    x1 = int(path[j+1][0] * cell + cell/2 + self.cam_x)
                    y1 = int(path[j+1][1] * cell + cell/2 + self.cam_y)
                    pygame.draw.line(self.screen, dim, (x0, y0), (x1, y1), thick_w)

            # Goal marker — use path destination to stay in sync
            gc, gr = path[-1] if path else ca["goal"]
            gx = int(gc * cell + cell/2 + self.cam_x)
            gy = int(gr * cell + cell/2 + self.cam_y)
            s  = max(3, cell // 3)
            pygame.draw.rect(self.screen, color, (gx-s, gy-s, s*2, s*2), 2)

            # Interpolated agent position — find the actual move span
            c0, r0 = path[step]
            if step < len(path) - 1:
                # Find start of current transit (first occurrence of this pos)
                transit_start = step
                while transit_start > 0 and path[transit_start - 1] == (c0, r0):
                    transit_start -= 1
                # Find end of transit (next different pos)
                transit_end = step + 1
                while transit_end < len(path) and path[transit_end] == (c0, r0):
                    transit_end += 1
                if transit_end < len(path):
                    c1, r1 = path[transit_end]
                    transit_len = transit_end - transit_start
                    progress = (local_t - transit_start) / max(1, transit_len)
                    progress = max(0.0, min(1.0, progress))
                    sx = (c0 + (c1 - c0) * progress) * cell + cell/2 + self.cam_x
                    sy = (r0 + (r1 - r0) * progress) * cell + cell/2 + self.cam_y
                else:
                    sx = c0 * cell + cell/2 + self.cam_x
                    sy = r0 * cell + cell/2 + self.cam_y
            else:
                sx = c0 * cell + cell/2 + self.cam_x
                sy = r0 * cell + cell/2 + self.cam_y

            pygame.draw.circle(self.screen, color, (int(sx), int(sy)), radius)
            pygame.draw.circle(self.screen, (255, 255, 255), (int(sx), int(sy)), radius, 1)
            if idx in self._collision_agents:
                pygame.draw.circle(self.screen, (255, 50, 50), (int(sx), int(sy)), radius + 3, 2)

    def _draw_collision_log(self):
        if not self._show_col_log:
            return
        ga_w  = self._grid_area_w()
        sh    = self.screen.get_height()
        panel_w  = 220
        row_h    = 14
        max_rows = 15
        entries  = self._collision_log[-max_rows:]
        n        = len(entries)
        header_h = 18
        panel_h  = header_h + n * row_h + 6

        px = 6
        py = sh - panel_h - 20   # bottom-left of grid area

        # Background
        bg = pygame.Surface((panel_w, panel_h), pygame.SRCALPHA)
        bg.fill((15, 15, 20, 210))
        self.screen.blit(bg, (px, py))
        pygame.draw.rect(self.screen, (80, 80, 100), (px, py, panel_w, panel_h), 1)

        # Header
        has_active = bool(self._collision_agents)
        hcol = (255, 80, 80) if has_active else (180, 100, 100)
        hdr  = self.font_s.render(
            f"Collision Log  [{self._collision_count}]", True, hcol)
        self.screen.blit(hdr, (px + 5, py + 3))

        # Entries (newest at bottom)
        for i, entry in enumerate(entries):
            ey   = py + header_h + i * row_h
            # Most recent entry highlighted
            col  = (255, 120, 120) if i == n - 1 else (160, 140, 140)
            txt  = self.font_s.render(entry, True, col)
            self.screen.blit(txt, (px + 6, ey))

    def _draw_astar_viz(self):
        if not self._astar_viz or not self._astar_viz_agents:
            return
        cell = self.cell
        ga_w = self._grid_area_w()
        ga_h = self.screen.get_height()

        overlay = pygame.Surface((ga_w, ga_h), pygame.SRCALPHA)

        for idx, cells in dict(self._astar_viz_agents).items():
            if not cells:
                continue
            color = agent_color(idx)
            for (c, r) in dict(cells):
                x = int(c * cell + self.cam_x)
                y = int(r * cell + self.cam_y)
                if x + cell < 0 or y + cell < 0 or x > ga_w or y > ga_h:
                    continue
                overlay.fill((*color, 40), (x, y, cell, cell))

        self.screen.blit(overlay, (0, 0))

        total = sum(len(v) for v in self._astar_viz_agents.values())
        txt = self.font_s.render(
            f"A* viz: {len(self._astar_viz_agents)} agents | {total} nodes  [V=close]",
            True, (255, 255, 100))
        self.screen.blit(txt, (8, 4))

    def _draw_box_preview(self):
        if not self._drag_painting or not self._box_start or not self._box_cur:
            return
        c0, r0 = self._box_start
        c1, r1 = self._box_cur
        x0 = int(min(c0, c1) * self.cell + self.cam_x)
        y0 = int(min(r0, r1) * self.cell + self.cam_y)
        w  = (abs(c1 - c0) + 1) * self.cell
        h  = (abs(r1 - r0) + 1) * self.cell
        if self.mode == MODE_FORBID:
            color = FORBID_CELL if self._drag_val else (200, 120, 60)
        elif self.mode == MODE_SPEED:
            color = SPEED_CELL if self._drag_val else (200, 120, 60)
        else:
            color = (80, 80, 80) if self._drag_val else (200, 120, 60)
        s = pygame.Surface((w, h), pygame.SRCALPHA)
        s.fill((*color, 80))
        self.screen.blit(s, (x0, y0))
        pygame.draw.rect(self.screen, color, (x0, y0, w, h), 2)

    def _draw_pibt_agents(self):
        cell   = self.cell
        radius = max(4, int(self.agent_radius * cell))
        frac   = min(1.0, self._pibt_frac)

        for idx, ca in self._cont_agents.items():
            color  = agent_color(idx)
            dim    = tuple(max(0, v - 100) for v in color)
            prev   = self._pibt_prev.get(idx)
            cur    = self._pibt_cur.get(idx, prev)
            goal   = ca["goal"]
            if prev is None:
                continue

            # Next-step arrow: prev → cur (only what PIBT actually guarantees)
            if cur != prev:
                nx0 = int(prev[0] * cell + cell/2 + self.cam_x)
                ny0 = int(prev[1] * cell + cell/2 + self.cam_y)
                nx1 = int(cur[0]  * cell + cell/2 + self.cam_x)
                ny1 = int(cur[1]  * cell + cell/2 + self.cam_y)
                pygame.draw.line(self.screen, dim, (nx0, ny0), (nx1, ny1),
                                 max(2, cell // 4))

            # Goal marker
            gx = int(goal[0] * cell + cell/2 + self.cam_x)
            gy = int(goal[1] * cell + cell/2 + self.cam_y)
            s  = max(3, cell // 3)
            pygame.draw.rect(self.screen, color, (gx-s, gy-s, s*2, s*2), 2)

            # Interpolated position
            sx = (prev[0] + (cur[0] - prev[0]) * frac) * cell + cell/2 + self.cam_x
            sy = (prev[1] + (cur[1] - prev[1]) * frac) * cell + cell/2 + self.cam_y

            # Agent circle
            pygame.draw.circle(self.screen, color, (int(sx), int(sy)), radius)
            pygame.draw.circle(self.screen, (255, 255, 255), (int(sx), int(sy)), radius, 1)
            if idx in self._collision_agents:
                pygame.draw.circle(self.screen, (255, 50, 50), (int(sx), int(sy)), radius + 3, 2)

            # Priority rank + stuck indicator
            rank  = self._pibt_rank.get(idx)
            stuck = ca.get("pibt_stuck", 0)
            if rank is not None and cell >= 10:
                r_col = (255, 60, 60) if stuck >= 4 else (255, 255, 255)
                r_lbl = self.font.render(str(rank), True, r_col)
                self.screen.blit(r_lbl, (int(sx) - r_lbl.get_width()  // 2,
                                          int(sy) - r_lbl.get_height() // 2))

    def _draw_cbs_progress(self, progress=None):
        prog = progress if progress is not None else self._cbs_progress
        if not prog:
            return
        cell  = self.cell
        ga_w  = self._grid_area_w()
        ga_h  = self.screen.get_height()

        # ── Candidate paths (dim, thin) ───────────────────────────────────────
        cand_paths = prog.get('paths', {})
        for idx, path in cand_paths.items():
            color = agent_color(self._valid_indices[idx]) if hasattr(self, '_valid_indices') and idx < len(self._valid_indices) else agent_color(idx)
            dim = tuple(max(0, v - 120) for v in color)
            if len(path) < 2:
                continue
            for j in range(1, len(path)):
                x0 = int(path[j-1][0] * cell + cell/2 + self.cam_x)
                y0 = int(path[j-1][1] * cell + cell/2 + self.cam_y)
                x1 = int(path[j][0]   * cell + cell/2 + self.cam_x)
                y1 = int(path[j][1]   * cell + cell/2 + self.cam_y)
                # Only draw if on screen
                if not (max(x0,x1) < 0 or min(x0,x1) > ga_w or
                        max(y0,y1) < 0 or min(y0,y1) > ga_h):
                    pygame.draw.line(self.screen, dim, (x0, y0), (x1, y1), 1)

        # ── Conflict marker (pulsing red ring) ────────────────────────────────
        cp = prog.get('conflict_pos')
        ct = prog.get('conflict_t', 0)
        if cp:
            cx = int(cp[0] * cell + cell/2 + self.cam_x)
            cy = int(cp[1] * cell + cell/2 + self.cam_y)
            pulse = int(abs(time.time() % 0.5 - 0.25) / 0.25 * 4) + 3
            pygame.draw.circle(self.screen, (255, 60, 60), (cx, cy), max(4, cell) + pulse, 2)
            pygame.draw.circle(self.screen, (255, 160, 60), (cx, cy), max(2, cell//2) + pulse//2, 1)
            # Show timestep so user can see it's a different conflict each time
            t_lbl = self.font_s.render(f"t={ct}", True, (255, 200, 80))
            self.screen.blit(t_lbl, (cx + max(4, cell) + 3, cy - 6))

    def _invalidate_grid_cache(self):
        self._grid_cache = None

    def _build_grid_cache(self):
        cell = self.cell
        w = self.grid.width * cell
        h = self.grid.height * cell
        surf = pygame.Surface((w, h))
        surf.fill(FREE_CELL)

        for c, r in self.grid.obstacles:
            surf.fill(OBS_CELL, (c * cell, r * cell, cell, cell))
        for c, r in self.grid.forbidden:
            surf.fill(FORBID_CELL, (c * cell, r * cell, cell, cell))
        for (c, r) in self.grid.speed_zones:
            surf.fill(SPEED_CELL, (c * cell, r * cell, cell, cell))

        # Grid lines
        if cell >= 3:
            for c in range(self.grid.width + 1):
                pygame.draw.line(surf, GRID_LINE, (c * cell, 0), (c * cell, h))
            for r in range(self.grid.height + 1):
                pygame.draw.line(surf, GRID_LINE, (0, r * cell), (w, r * cell))

        self._grid_cache = surf
        self._grid_cache_cell = cell

    def _draw_grid(self):
        if (self._grid_cache is None or
                getattr(self, '_grid_cache_cell', None) != self.cell):
            self._build_grid_cache()

        ga_w = self._grid_area_w()
        ga_h = self.screen.get_height()
        cx = int(self.cam_x)
        cy = int(self.cam_y)

        # source rect on the cache surface
        sx = max(0, -cx)
        sy = max(0, -cy)
        # destination on screen
        dx = max(0, cx)
        dy = max(0, cy)
        sw = min(self._grid_cache.get_width() - sx, ga_w - dx)
        sh = min(self._grid_cache.get_height() - sy, ga_h - dy)
        if sw > 0 and sh > 0:
            self.screen.blit(self._grid_cache, (dx, dy), (sx, sy, sw, sh))

    def _draw_paths(self):
        if not self.paths:
            return
        line_w = max(2, self.cell // 4)   # CBS-solved: always thick
        for i, path in self.paths.items():
            color = agent_color(i)
            dim   = tuple(max(0, v - 80) for v in color)

            if len(path) < 2:
                continue
            for j in range(1, len(path)):
                x0, y0 = self._grid_to_screen(*path[j - 1])
                x1, y1 = self._grid_to_screen(*path[j])
                cx0 = x0 + self.cell // 2
                cy0 = y0 + self.cell // 2
                cx1 = x1 + self.cell // 2
                cy1 = y1 + self.cell // 2
                pygame.draw.line(self.screen, dim, (cx0, cy0), (cx1, cy1), line_w)

    def _draw_agents(self):
        cell   = self.cell
        # Visual radius: agent_radius cells → pixels, clamped to reasonable range
        radius = max(4, int(self.agent_radius * cell))

        for i, agent in enumerate(self.agents):
            color  = agent_color(i)
            is_sel = (i == self.sel)

            # ── Animated position from path ──────────────────────────────────
            if i in self.paths:
                path = self.paths[i]
                step = min(self.sim_step, len(path) - 1)
                if self.sim_run and step < len(path) - 1:
                    frac    = self.sim_t - int(self.sim_t)
                    c0, r0  = path[step]
                    c1, r1  = path[step + 1]
                    sx = (c0 + (c1 - c0) * frac) * cell + cell / 2 + self.cam_x
                    sy = (r0 + (r1 - r0) * frac) * cell + cell / 2 + self.cam_y
                else:
                    c0, r0  = path[step]
                    sx = c0 * cell + cell / 2 + self.cam_x
                    sy = r0 * cell + cell / 2 + self.cam_y
                pygame.draw.circle(self.screen, color, (int(sx), int(sy)), radius)
                pygame.draw.circle(self.screen, (255, 255, 255), (int(sx), int(sy)), radius, 1)

                # goal marker (hollow square)
                gx, gy = self._grid_to_screen(*agent["goal"])
                s = max(3, cell // 3)
                cx = gx + cell // 2
                cy = gy + cell // 2
                pygame.draw.rect(self.screen, color, (cx - s, cy - s, s * 2, s * 2), 2)

            else:
                # ── Static start marker ──────────────────────────────────────
                if agent["start"]:
                    sx, sy = self._grid_to_screen(*agent["start"])
                    cx, cy = sx + cell // 2, sy + cell // 2
                    pygame.draw.circle(self.screen, color, (cx, cy), radius)
                    ring_color = (255, 255, 255) if is_sel else (160, 160, 160)
                    pygame.draw.circle(self.screen, ring_color, (cx, cy), radius + (2 if is_sel else 0), 2)

                # ── Static goal marker ───────────────────────────────────────
                if agent["goal"]:
                    gx, gy = self._grid_to_screen(*agent["goal"])
                    cx, cy = gx + cell // 2, gy + cell // 2
                    s = max(3, cell // 3)
                    pygame.draw.rect(self.screen, color, (cx - s, cy - s, s * 2, s * 2), 2)
                    if is_sel:
                        pygame.draw.rect(self.screen, (255, 255, 255),
                                         (cx - s - 2, cy - s - 2, s * 2 + 4, s * 2 + 4), 1)

            # Agent label (only if cell big enough)
            if cell >= 14:
                lbl = self.font_s.render(f"A{i+1}", True, color)
                if agent["start"] and i not in self.paths:
                    sx, sy = self._grid_to_screen(*agent["start"])
                    self.screen.blit(lbl, (sx + 1, sy + 1))

    def _draw_adj_row(self, x0, sy, bar_w, label, attr_minus, attr_plus):
        bw = 18
        self.screen.blit(self.font_s.render(label, True, DIM), (x0+8, sy))
        for lbl, dx in [("-", bar_w - bw*2 - 2), ("+", bar_w - bw)]:
            r = pygame.Rect(x0+8+dx, sy-1, bw, 14)
            hov = r.collidepoint(pygame.mouse.get_pos())
            pygame.draw.rect(self.screen, BTN_HOVER if hov else BTN_NORMAL, r, border_radius=3)
            self.screen.blit(self.font_s.render(lbl, True, TEXT), (r.x+5, r.y+1))
            setattr(self, attr_minus if lbl == "-" else attr_plus, r)

    def _draw_sidebar(self):
        sw = self.screen.get_width()
        sh = self.screen.get_height()
        x0 = sw - SIDEBAR_W

        pygame.draw.rect(self.screen, SIDEBAR_BG, (x0, 0, SIDEBAR_W, sh))
        pygame.draw.line(self.screen, SIDEBAR_EDGE, (x0, 0), (x0, sh), 2)

        # Title
        t = self.font_b.render("MAPF  Simulator", True, TEXT)
        self.screen.blit(t, (x0 + 8, 8))

        # Map size in meters
        res = self.grid.resolution
        map_w_m = self.grid.width * res
        map_h_m = self.grid.height * res
        sz = self.font_s.render(f"{map_w_m:.1f}x{map_h_m:.1f}m  ({self.grid.width}x{self.grid.height}cell  {res}m/cell)", True, DIM)
        self.screen.blit(sz, (x0 + 8, 22))

        # Buttons
        for b in self._btns:
            b.draw(self.screen, self.font_s)

        # Agent radius control
        ry = self._y_radius
        bar_w = SIDEBAR_W - 20

        # Radius bar
        ry = self._y_radius
        res = self.grid.resolution
        radius_m = self.agent_radius * res
        diam_m   = radius_m * 2
        self.screen.blit(self.font_s.render(f"R:{radius_m:.2f}m  D:{diam_m:.2f}m  click/scroll", True, DIM), (x0+8, ry))
        r_max_cell = 10.0 / self.grid.resolution
        r_min, r_max = 0.5, r_max_cell
        filled_r = int(bar_w * (self.agent_radius - r_min) / max(1, r_max - r_min))
        pygame.draw.rect(self.screen, (55, 55, 70),   (x0+8, ry+13, bar_w, 8), border_radius=4)
        pygame.draw.rect(self.screen, (180, 120, 60), (x0+8, ry+13, filled_r, 8), border_radius=4)
        self._rect_radius_bar = pygame.Rect(x0+8, ry, bar_w, 22)

        # Speed bar
        sy = self._y_speed
        self.screen.blit(self.font_s.render(f"Sim: {self.sim_speed:.1f}x realtime", True, DIM), (x0+8, sy))
        filled = int(bar_w * (self.sim_speed - 0.1) / 4.9)
        pygame.draw.rect(self.screen, (55, 55, 70), (x0+8, sy+13, bar_w, 8), border_radius=4)
        pygame.draw.rect(self.screen, BTN_ACTIVE,   (x0+8, sy+13, filled, 8), border_radius=4)

        # Timeout row
        ty = sy + 30
        self._draw_adj_row(x0, ty, bar_w, f"Timeout: {self.solve_timeout:.0f}s",
                           '_rect_timeout_minus', '_rect_timeout_plus')

        # Max-nodes row
        self._draw_adj_row(x0, ty+18, bar_w, f"Max nodes: {self.max_nodes}",
                           '_rect_nodes_minus', '_rect_nodes_plus')

        # RHCR W / H rows
        wy = self._y_rhcr_wh
        self._draw_adj_row(x0, wy,    bar_w, f"RHCR  W: {self._rhcr_window}",
                           '_rect_rhcr_w_minus', '_rect_rhcr_w_plus')
        self._draw_adj_row(x0, wy+18, bar_w, f"RHCR  H: {self._rhcr_h}",
                           '_rect_rhcr_h_minus', '_rect_rhcr_h_plus')

        # Agent list (scrollable)
        ay = self.b_call.rect.bottom + 12
        list_h     = sh - ay - 80          # available height for list
        row_h      = 15
        visible    = max(1, list_h // row_h)

        # Auto-scroll so selected agent is always visible
        self._agent_list_scroll = max(0, min(
            self._agent_list_scroll,
            max(0, len(self.agents) - visible)
        ))
        if self.sel < self._agent_list_scroll:
            self._agent_list_scroll = self.sel
        elif self.sel >= self._agent_list_scroll + visible:
            self._agent_list_scroll = self.sel - visible + 1

        total = len(self.agents)
        header = f"Agents: {total}/{MAX_AGENTS}"
        pygame.draw.rect(self.screen, (35, 45, 55), (x0 + 4, ay - 1, SIDEBAR_W - 8, 14), border_radius=3)
        self.screen.blit(self.font_s.render(header, True, (160, 200, 220)), (x0+8, ay))
        ay += 14
        self._agent_list_top = ay   # store for scroll hit-test
        self._agent_speed_minus = {}
        self._agent_speed_plus = {}

        for slot in range(visible):
            i = self._agent_list_scroll + slot
            if i >= total:
                break
            a      = self.agents[i]
            color  = agent_color(i)
            is_sel = (i == self.sel)
            ry     = ay + slot * row_h
            if is_sel:
                pygame.draw.rect(self.screen, (60, 65, 92),
                                 (x0+6, ry-1, SIDEBAR_W-12, row_h), border_radius=3)
            pygame.draw.rect(self.screen, color, (x0+8, ry+3, 8, 8))
            s_s = f"({a['start'][0]},{a['start'][1]})" if a["start"] else "?"
            g_s = f"({a['goal'][0]},{a['goal'][1]})"   if a["goal"]  else "?"
            spd = a.get("tpc", 1)

            # Speed display with +/- buttons
            ms = BASE_SPEED / spd
            bw = 14
            spd_txt = self.font_s.render(f"{ms:.2f}m/s", True, (255, 200, 80))
            txt_w = spd_txt.get_width()
            total_w = bw + 4 + txt_w + 4 + bw
            spd_x = x0 + SIDEBAR_W - total_w - 8

            # - button
            minus_r = pygame.Rect(spd_x, ry, bw, row_h - 1)
            hov_m = minus_r.collidepoint(pygame.mouse.get_pos())
            pygame.draw.rect(self.screen, BTN_HOVER if hov_m else (60, 55, 35), minus_r, border_radius=2)
            self.screen.blit(self.font_s.render("-", True, (255, 200, 80)), (minus_r.x + 4, ry))

            # speed text
            self.screen.blit(spd_txt, (spd_x + bw + 4, ry + 1))

            # + button
            plus_r = pygame.Rect(spd_x + bw + 4 + txt_w + 4, ry, bw, row_h - 1)
            hov_p = plus_r.collidepoint(pygame.mouse.get_pos())
            pygame.draw.rect(self.screen, BTN_HOVER if hov_p else (60, 55, 35), plus_r, border_radius=2)
            self.screen.blit(self.font_s.render("+", True, (255, 200, 80)), (plus_r.x + 3, ry))

            self._agent_speed_minus[i] = minus_r
            self._agent_speed_plus[i] = plus_r

            arr = self._cont_agents.get(i, {}).get("arrivals", 0) if self._cont_mode else 0
            arr_s = f" :{arr}" if self._cont_mode else ""
            row_txt = self.font_s.render(f" A{i+1}{arr_s}", True, TEXT if is_sel else DIM)
            self.screen.blit(row_txt, (x0+19, ry))

        # Scroll indicator
        if total > visible:
            bar_x  = x0 + SIDEBAR_W - 6
            bar_top = ay
            bar_bot = ay + visible * row_h
            bar_h   = bar_bot - bar_top
            thumb_h = max(12, bar_h * visible // total)
            thumb_y = bar_top + (bar_h - thumb_h) * self._agent_list_scroll // max(1, total - visible)
            pygame.draw.rect(self.screen, (50, 50, 65),  (bar_x, bar_top, 4, bar_h), border_radius=2)
            pygame.draw.rect(self.screen, (100, 100, 130), (bar_x, thumb_y, 4, thumb_h), border_radius=2)

        # Section labels with background
        for attr, text, col, bg_col in [
            ('_y_agent_label',    ' Agent',          (180, 230, 190), (35, 50, 40)),
            ('_y_param_label',    ' Parameters',     (220, 210, 160), (50, 45, 30)),
            ('_y_research_label',      ' Research',       (160, 160, 200), (38, 38, 50)),
            ('_y_realtime_label',      ' Realtime',       (200, 220, 140), (45, 50, 30)),
            ('_y_production_label',    ' Production',     (140, 220, 180), (30, 50, 40)),
            ('_y_map_label',           ' Map / Data',     (190, 180, 220), (40, 38, 55)),
        ]:
            sy = getattr(self, attr)
            pygame.draw.rect(self.screen, bg_col, (x0 + 4, sy - 1, SIDEBAR_W - 8, 14), border_radius=3)
            pygame.draw.line(self.screen, (*col, 100), (x0 + 4, sy + 13), (x0 + SIDEBAR_W - 4, sy + 13), 1)
            lbl = self.font_s.render(text, True, col)
            self.screen.blit(lbl, (x0 + 8, sy))

        # Tree line: CBS → RHCR
        tree_col = (80, 100, 130)
        lx = x0 + 8
        if hasattr(self, 'b_auto_rhcr'):
            pygame.draw.line(self.screen, tree_col, (lx, self.b_auto_cbs.rect.bottom),
                             (lx, self.b_auto_rhcr.rect.centery), 1)
            pygame.draw.line(self.screen, tree_col, (lx, self.b_auto_rhcr.rect.centery),
                             (lx + 8, self.b_auto_rhcr.rect.centery), 1)

        # Slow Zone speed row
        self._draw_adj_row(x0, self._y_slow_tpc, bar_w,
                           f"Slow: {BASE_SPEED / self._zone_speed_tpc:.2f} m/s",
                           '_rect_zone_tpc_minus', '_rect_zone_tpc_plus')

        # SOLVE 버튼 라벨 동적 변경
        self.b_solve.label = "CANCEL" if self._solving else "SOLVE CBS"
        self.b_solve.active = self._solving

        # Status bar
        if self._solving:
            elapsed   = time.time() - self._solve_start
            remaining = max(0.0, self.solve_timeout - elapsed)
            nodes     = self._cbs_progress.get('nodes', 0)
            opensize  = self._cbs_progress.get('open_size', 0)
            status = f"Solving... {elapsed:.1f}s  (limit:{remaining:.0f}s)"
            col    = (100, 220, 100)
            # Extra CBS stats line
            stats_txt = self.font_s.render(
                f"nodes:{nodes}  open:{opensize}", True, (140, 200, 140))
            self.screen.blit(stats_txt, (x0+8, sh-60))
            prog_lbl = self.font_s.render(
                f"[Tab] viz: {'ON' if self._show_progress else 'OFF'}", True, DIM)
            self.screen.blit(prog_lbl, (x0+8, sh-72))
        elif self._cont_mode:
            if self._cont_cbs_run:
                nodes  = self._cont_progress.get('nodes', 0)
                opens  = self._cont_progress.get('open_size', 0)
                status = f"Auto CBS: computing...  nodes:{nodes} open:{opens}"
                col    = (100, 220, 100)
            else:
                mode_s = ["A*", "CBS", "RHCR", "PBS", "PIBT"][self._cont_replan_mode]
                status = f"Auto({mode_s}): {self._cont_total} arrivals  ({len(self._cont_agents)} agents)"
                col    = (100, 220, 180)
        elif self.sim_run:
            mx     = max((len(p) for p in self.paths.values()), default=1)
            status = f"Step {self.sim_step}/{mx-1}"
            col    = (100, 180, 255)
        elif self.paths:
            status = "Done."
            col    = (100, 220, 100)
        else:
            status = ""
            col    = TEXT

        if status:
            self.screen.blit(self.font_s.render(status, True, col), (x0+8, sh-44))

        # Mode / hint
        sub_str = ("", " [start]", " [goal]")[self.edit_sub]
        mode_label = {"obstacle": "Draw", "erase": "Erase", "agent": "Agent"}.get(self.mode, self.mode)
        m_str   = f"Mode: {mode_label}{sub_str}"
        self.screen.blit(self.font_s.render(m_str,                True, (140, 140, 200)), (x0+8, sh-28))
        self.screen.blit(self.font_s.render("MMB=pan  Wheel=zoom", True, DIM),             (x0+8, sh-14))

    def _draw_stat_bar(self):
        sh   = self.screen.get_height()
        ga_w = self._grid_area_w()
        bar_h = 16
        y     = sh - bar_h

        pygame.draw.rect(self.screen, (20, 20, 24), (0, y, ga_w, bar_h))

        col_str = f"collisions:{self._collision_count}"
        col_color = (255, 80, 80) if self._collision_agents else (140, 140, 155)
        parts = [f"FPS:{self._stat_fps:5.1f}", f"frame:{self._stat_frame_ms:5.1f}ms", col_str]

        if self._cont_mode:
            if self._cont_replan_mode in (1, 2, 3) and self._cont_cbs_run:
                if self._cont_replan_mode == 3:
                    agent  = self._cont_progress.get('agent', '?')
                    total  = self._cont_progress.get('total', '?')
                    parts.append(f"PBS planning agent {agent}/{total}")
                else:
                    nodes = self._cont_progress.get('nodes', 0)
                    parts.append(f"CBS nodes:{nodes}")
            elif self._cont_replan_mode == 2:
                time_to_next = max(0.0, self._rhcr_h - (self._rhcr_global_t - self._rhcr_last_t))
                parts.append(f"RHCR W={self._rhcr_window} H={self._rhcr_h} nxt:{time_to_next:.1f}")
            else:
                status = "A* running..." if self._astar_running else f"last A*:{self._stat_replan_ms:.0f}ms"
                parts.append(status)
                parts.append(f"replan:{self._stat_replan_ps:.1f}/s")
            parts.append(f"arrivals:{self._cont_total}")
            parts.append(f"throughput:{self._throughput:.0f} tasks/min")

        if self._solving:
            nodes = self._cbs_progress.get('nodes', 0)
            opens = self._cbs_progress.get('open_size', 0)
            parts.append(f"CBS nodes:{nodes}  open:{opens}")

        bar_col = (220, 80, 80) if self._collision_agents else (140, 140, 155)
        txt = self.font_s.render("  |  ".join(parts), True, bar_col)
        self.screen.blit(txt, (6, y + 2))

    def _draw_tooltip(self):
        mx, my = pygame.mouse.get_pos()
        for b in self._btns:
            if b.tip and b.rect.collidepoint(mx, my):
                txt = self.font_s.render(b.tip, True, (240, 240, 240))
                tw, th = txt.get_size()
                tx = min(mx, self.screen.get_width() - tw - 10)
                ty = b.rect.bottom + 4
                bg = pygame.Surface((tw + 8, th + 4), pygame.SRCALPHA)
                bg.fill((20, 20, 30, 220))
                self.screen.blit(bg, (tx - 4, ty - 2))
                pygame.draw.rect(self.screen, (100, 100, 130), (tx - 4, ty - 2, tw + 8, th + 4), 1, border_radius=3)
                self.screen.blit(txt, (tx, ty))
                break

    def _draw_msg(self):
        txt = self.font.render(self.msg, True, (255, 255, 100))
        tw, th = txt.get_size()
        ga_w = self._grid_area_w()
        x = (ga_w - tw) // 2
        y = self.screen.get_height() - 36
        bg = pygame.Surface((tw + 14, th + 8), pygame.SRCALPHA)
        bg.fill(MSG_BG)
        self.screen.blit(bg, (x - 7, y - 4))
        self.screen.blit(txt, (x, y))

    # ── Event loop ─────────────────────────────────────────────────────────────
    def run(self):
        running = True
        while running:
            dt = self.clock.tick(60) / 1000.0

            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    self._save_config()
                    if self._sim_log:
                        self._log("sim_stop", total_arrivals=self._cont_total,
                                  collisions=self._collision_count)
                        self._flush_log()
                    running = False

                elif ev.type == pygame.MOUSEBUTTONDOWN:
                    # Radius bar: left=+0.5, right=-0.5
                    _r_bar = getattr(self, '_rect_radius_bar', None)
                    if _r_bar and _r_bar.collidepoint(ev.pos) and ev.button == 1:
                        try:
                            import tkinter as _tk
                            from tkinter import simpledialog as _sd
                            _root = _tk.Tk(); _root.withdraw()
                            cur_m = self.agent_radius * self.grid.resolution
                            val = _sd.askfloat(
                                "Robot Radius",
                                f"반지름 (m) 입력:\n현재: {cur_m:.3f}m (지름 {cur_m*2:.3f}m)",
                                initialvalue=round(cur_m, 3),
                                minvalue=0.01, maxvalue=10.0,
                            )
                            _root.destroy()
                            if val is not None:
                                self.agent_radius = round(val / self.grid.resolution, 1)
                                self.agent_radius = max(0.5, self.agent_radius)
                                self._save_config()
                        except Exception:
                            pass

                    # Timeout / nodes / RHCR W / RHCR H mini-buttons
                    if ev.button == 1:
                        for attr, lo, hi, step, target in [
                            ('_rect_timeout_minus', 5.0,   300.0, -10,   'solve_timeout'),
                            ('_rect_timeout_plus',  5.0,   300.0, +10,   'solve_timeout'),
                            ('_rect_nodes_minus',   500,  100000, -500,  'max_nodes'),
                            ('_rect_nodes_plus',    500,  100000, +500,  'max_nodes'),
                        ]:
                            r = getattr(self, attr, None)
                            if r and r.collidepoint(ev.pos):
                                setattr(self, target,
                                        max(lo, min(hi, getattr(self, target) + step)))
                        # RHCR W buttons
                        for attr, delta in [('_rect_rhcr_w_minus', -1), ('_rect_rhcr_w_plus', +1)]:
                            r = getattr(self, attr, None)
                            if r and r.collidepoint(ev.pos):
                                self._rhcr_window = max(self._rhcr_h, min(200, self._rhcr_window + delta))
                                self._save_config()
                        # RHCR H buttons — clamp min = max(agent tpc)
                        for attr, delta in [('_rect_rhcr_h_minus', -1), ('_rect_rhcr_h_plus', +1)]:
                            r = getattr(self, attr, None)
                            if r and r.collidepoint(ev.pos):
                                mt = max(2, self._max_tpc())
                                self._rhcr_h = max(mt, min(self._rhcr_window, self._rhcr_h + delta))
                                self._rhcr_window = max(self._rhcr_window, self._rhcr_h)
                                self._save_config()
                        # Zone TPC buttons
                        for attr, delta in [('_rect_zone_tpc_minus', -1), ('_rect_zone_tpc_plus', +1)]:
                            r = getattr(self, attr, None)
                            if r and r.collidepoint(ev.pos):
                                self._zone_speed_tpc = max(2, min(200, self._zone_speed_tpc - delta))
                    # Agent speed +/- buttons
                    _spd_clicked = False
                    if ev.button == 1:
                        for _ai, _sr in self._agent_speed_minus.items():
                            if _sr.collidepoint(ev.pos):
                                self._adjust_agent_speed(_ai, -1)
                                _spd_clicked = True; break
                        if not _spd_clicked:
                            for _ai, _sr in self._agent_speed_plus.items():
                                if _sr.collidepoint(ev.pos):
                                    self._adjust_agent_speed(_ai, +1)
                                    _spd_clicked = True; break
                    # Sidebar buttons
                    handled = _spd_clicked or any(b.on_event(ev) for b in self._btns)
                    if not handled:
                        if ev.button in (1, 3):
                            self._on_grid_click(ev.pos[0], ev.pos[1], ev.button)
                        elif ev.button == 2:
                            self._panning  = True
                            self._pan_orig = ev.pos

                elif ev.type == pygame.MOUSEBUTTONUP:
                    if self._drag_painting and ev.button in (1, 3):
                        self._apply_box()
                    self._drag_painting = False
                    if ev.button == 2:
                        self._panning = False

                elif ev.type == pygame.MOUSEMOTION:
                    for b in self._btns:
                        b.on_event(ev)
                    if self._panning:
                        dx = ev.pos[0] - self._pan_orig[0]
                        dy = ev.pos[1] - self._pan_orig[1]
                        self.cam_x    += dx
                        self.cam_y    += dy
                        self._pan_orig = ev.pos
                        self._clamp_cam()
                    elif self._drag_painting:
                        self._on_grid_drag(ev.pos[0], ev.pos[1])

                elif ev.type == pygame.MOUSEWHEEL:
                    mx, my = pygame.mouse.get_pos()
                    if mx < self._grid_area_w():          # zoom on grid area
                        old = self.cell
                        self.cell = max(CELL_MIN, min(CELL_MAX, self.cell + ev.y))
                        gx = (mx - self.cam_x) / old
                        gy = (my - self.cam_y) / old
                        self.cam_x = mx - gx * self.cell
                        self.cam_y = my - gy * self.cell
                        self._clamp_cam()
                    else:
                        top = getattr(self, '_agent_list_top', 9999)
                        if my >= top:                      # agent list area → scroll list
                            self._agent_list_scroll = max(
                                0, min(len(self.agents) - 1,
                                       self._agent_list_scroll - ev.y)
                            )
                        else:
                            _r_bar = getattr(self, '_rect_radius_bar', None)
                            if _r_bar and _r_bar.collidepoint(mx, my):
                                self.agent_radius = round(max(0.5, self.agent_radius + ev.y * 0.5), 1)
                                self._save_config()
                            else:
                                self.sim_speed = round(
                                    max(0.1, min(5.0, self.sim_speed + ev.y * 0.1)), 1)

                elif ev.type == pygame.VIDEORESIZE:
                    self._screen_w = ev.w
                    self._screen_h = ev.h
                    self._rebuild_btns()

                elif ev.type == pygame.KEYDOWN:
                    if ev.key == pygame.K_v:
                        self._toggle_astar_viz()
                    elif ev.key == pygame.K_TAB:
                        self._show_progress = not self._show_progress
                    elif ev.key == pygame.K_l:
                        self._show_col_log = not self._show_col_log

            self.update(dt)
            self.draw()

        pygame.quit()
