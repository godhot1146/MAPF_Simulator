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
CELL_MIN     =  8
CELL_MAX     = 40

MODE_OBS   = "obstacle"
MODE_ERASE = "erase"
MODE_AGNT  = "agent"

SUB_NONE  = 0
SUB_START = 1
SUB_GOAL  = 2


# ── Tiny button helper ─────────────────────────────────────────────────────────
class Btn:
    def __init__(self, rect, label, cb, toggle=False):
        self.rect   = pygame.Rect(rect)
        self.label  = label
        self.cb     = cb
        self.toggle = toggle
        self.active = False
        self._hover = False

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
        self.sim_speed    = float(c.get("sim_speed",    5.0))
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

        self._drag_painting    = False
        self._drag_val         = True
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
        self._rhcr_window      = int(c.get("rhcr_window", 12))  # W: planning horizon (CBS depth)
        self._rhcr_h           = int(c.get("rhcr_h",      5))   # H: execution steps before replan (H <= W)
        self._rhcr_global_t    = 0.0   # cumulative sim steps in cont mode
        self._rhcr_last_t      = 0.0   # global_t at last RHCR replan
        self._rhcr_arrived     = set() # agents counted as arrived (waiting for replan)

        self.msg       = ""
        self.msg_ticks = 0

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

        self.font   = pygame.font.SysFont("consolas", 13)
        self.font_s = pygame.font.SysFont("consolas", 11)
        self.font_b = pygame.font.SysFont("consolas", 14, bold=True)

        self._screen_w = self.grid.width * self.cell + SIDEBAR_W
        self._screen_h = max(self.grid.height * self.cell, 520)
        self.screen = pygame.display.set_mode(
            (self._screen_w, self._screen_h), pygame.RESIZABLE
        )

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

    # ── Button layout ──────────────────────────────────────────────────────────
    def _rebuild_btns(self):
        x0 = self._grid_area_w() + 8
        w  = SIDEBAR_W - 16
        hw = w // 2 - 2
        h  = 26
        g  = 5
        y  = 32

        def B(rect, label, cb, toggle=False):
            return Btn(rect, label, cb, toggle)

        self.b_obs   = B((x0,      y, hw, h), "Draw  [1]",  lambda: self._set_mode(MODE_OBS),   True)
        self.b_era   = B((x0+hw+4, y, hw, h), "Erase [3]",  lambda: self._set_mode(MODE_ERASE), True); y += h+g
        self.b_agnt  = B((x0, y, w,      h), "Agent    [2]",  lambda: self._set_mode(MODE_AGNT), True); y += h+g*2
        self.b_add   = B((x0,       y, hw, h), "+Agent",        self._add_agent)
        self.b_del   = B((x0+hw+4,  y, hw, h), "-Agent",        self._del_agent); y += h+g
        self.b_start = B((x0,       y, hw, h), "SetStart [s]",  self._toggle_start, True)
        self.b_goal  = B((x0+hw+4,  y, hw, h), "SetGoal  [g]",  self._toggle_goal,  True); y += h+g*2
        # Agent radius control  (label 14px + bar 10px + gap)
        self._y_radius = y;                                                                 y += 30+g
        # Sim speed control   (label 14px + bar 10px + gap)
        self._y_speed = y;                                                                  y += 30+g
        # Timeout row         (label 14px + gap)
        y += 18
        # Max-nodes row       (label 14px + gap)
        y += 18+g
        self.b_auto_astar = B((x0, y, w, h), "Auto: A*   [a]", lambda: self._start_auto(0), True); y += h+g
        self.b_auto_cbs   = B((x0, y, w, h), "Auto: CBS  [b]", lambda: self._start_auto(1), True); y += h+g
        self.b_auto_rhcr  = B((x0, y, w, h), "Auto: RHCR [h]", lambda: self._start_auto(2), True); y += h+g
        self.b_solve  = B((x0, y, w, h), "SOLVE CBS [Space]", self._solve_or_cancel);               y += h+g
        self.b_rsim  = B((x0, y, w, h), "Reset Sim  [r]", self._reset_sim);                y += h+g*2
        self.b_save  = B((x0,       y, hw, h), "Save Map", self._save_map)
        self.b_load  = B((x0+hw+4,  y, hw, h), "Load Map", self._load_map);                y += h+g
        self.b_new   = B((x0, y, w, h), "New Map...",     self._new_map);                  y += h+g
        self.b_rand  = B((x0, y, w, h), "Random Agents",  self._random_agents);             y += h+g
        self.b_clag  = B((x0, y, w, h), "Clear Agents",   self._clear_agents);             y += h+g
        self.b_call  = B((x0, y, w, h), "Clear All",      self._clear_all)

        self._btns = [
            self.b_obs, self.b_era, self.b_agnt,
            self.b_add, self.b_del,
            self.b_start, self.b_goal,
            self.b_auto_astar, self.b_auto_cbs, self.b_auto_rhcr,
            self.b_solve, self.b_rsim,
            self.b_save, self.b_load,
            self.b_new, self.b_rand, self.b_clag, self.b_call,
        ]
        self._refresh_btn_state()

    def _refresh_btn_state(self):
        self.b_obs.active      = (self.mode == MODE_OBS)
        self.b_era.active      = (self.mode == MODE_ERASE)
        self.b_agnt.active     = (self.mode == MODE_AGNT)
        self.b_start.active    = (self.edit_sub == SUB_START)
        self.b_goal.active     = (self.edit_sub == SUB_GOAL)
        self.b_auto_astar.active = self._cont_mode and self._cont_replan_mode == 0
        self.b_auto_cbs.active   = self._cont_mode and self._cont_replan_mode == 1
        self.b_auto_rhcr.active  = self._cont_mode and self._cont_replan_mode == 2

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

    def _record_arrival(self):
        """Call each time any agent completes a task. Updates throughput."""
        now = time.time()
        self._throughput_window.append(now)
        # Keep only last 60 seconds
        cutoff = now - 60.0
        self._throughput_window = [t for t in self._throughput_window if t > cutoff]
        # tasks/min = count in last 60s
        self._throughput = len(self._throughput_window)

    def _show(self, msg, secs=3):
        self.msg       = msg
        self.msg_ticks = secs * 60

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

    def _add_agent(self):
        if len(self.agents) >= MAX_AGENTS:
            self._show(f"Max {MAX_AGENTS} agents.")
            return
        self.agents.append({"start": None, "goal": None})
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
            self._add_or_remove_obstacle(c, r, placing)
            self._drag_painting = True
            self._drag_val      = placing

        elif self.mode == MODE_ERASE:
            self._add_or_remove_obstacle(c, r, False)
            self._drag_painting = True
            self._drag_val      = False

        elif self.mode == MODE_AGNT:
            if not self.agents:
                self._show("Add an agent first.")
                return
            agent = self.agents[self.sel]

            if self.edit_sub == SUB_START:
                if self.grid.is_obstacle(c, r):
                    self._show("Cannot place start on obstacle.")
                    return
                agent["start"] = (c, r)
                self.edit_sub  = SUB_GOAL
                self._refresh_btn_state()
                self._show("Start set – now click goal.")

            elif self.edit_sub == SUB_GOAL:
                if self.grid.is_obstacle(c, r):
                    self._show("Cannot place goal on obstacle.")
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
        if gp and self.mode == MODE_OBS:
            self._add_or_remove_obstacle(gp[0], gp[1], self._drag_val)

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
        result = solve_cbs(
            self.grid, valid_agents,
            max_time=400, agent_radius=agent_radius,
            stop_event=self._stop_event,
            progress=self._cbs_progress,
            max_nodes=self.max_nodes,
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
                self._save_config(path)
                self._reset_sim()
                self._clamp_cam()
                self._rebuild_btns()
                self._show(f"Loaded: {os.path.basename(path)}")
        except Exception as e:
            self._show(f"Load error: {e}")

    def _clear_agents(self):
        self.agents = []
        self.sel    = 0
        self._reset_sim()

    def _clear_all(self):
        self.grid   = Grid.empty(self.grid.width, self.grid.height)
        self.agents = []
        self.sel    = 0
        self._reset_sim()

    # ── Continuous / Auto mode ────────────────────────────────────────────────

    def _stop_cont_mode(self):
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
            }

        self._cont_mode = True   # set BEFORE _reset_sim so cont_agents aren't cleared
        self._rhcr_arrived.clear()
        self._rhcr_global_t = 0.0
        if self._cont_replan_mode == 2:
            # Trigger first replan immediately so RHCR starts coordinated
            self._rhcr_last_t = -float(self._rhcr_h)
        else:
            self._rhcr_last_t = 0.0
        self._reset_sim()
        mode_names = ["A*", "CBS", "RHCR"]
        self._show(f"Auto({mode_names[self._cont_replan_mode]}): {len(self._cont_agents)} agents running.")

    def _pick_new_goal(self, idx):
        """Pick a spread-out random goal for agent idx, avoiding other agents/goals."""
        import random, math
        safe     = self._safe_cache or set()
        min_dist = self.agent_radius * 2 + 0.5
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

                offsets = self._body_offsets()
                horizon = self._CONSTRAINT_HORIZON
                vc = set()
                for oi, oa in self._cont_agents.items():
                    if oi == idx:
                        continue
                    op  = oa["path"]
                    ot  = min(int(oa["local_t"]), len(op) - 1)
                    frozen = (ot >= len(op) - 1)
                    if frozen:
                        # Frozen agent: treat as static obstacle for all future steps
                        c, r = op[ot]
                        for future_dt in range(horizon):
                            for dc, dr in offsets:
                                vc.add((c + dc, r + dr, future_dt + 1))
                        continue
                    end = min(ot + horizon, len(op))
                    for dt, pi in enumerate(range(ot, end)):
                        c, r = op[pi]
                        for dc, dr in offsets:
                            vc.add((c + dc, r + dr, dt + 1))

                from astar import astar
                new_path = astar(self.grid, start, goal, vc, set(),
                                 max_time=200, safe_positions=safe)
                if new_path is None:
                    new_path = astar(self.grid, start, goal, safe_positions=safe)
                if new_path is None:
                    new_path = [start]
                results[idx] = new_path

            elapsed = (time.time() - t0) * 1000
            self._astar_results  = results
            self._stat_replan_ms  = elapsed
            self._stat_replan_cnt += len(results)
            self._astar_running   = False

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

        self._stop_event.clear()   # reset any leftover cancel from manual solver
        self._cont_cbs_run  = True
        self._cont_cbs_snap = snap_local_t
        self._cont_cbs_frac = snap_frac
        self._cont_progress.clear()

        rhcr_horizon = self._rhcr_window if self._cont_replan_mode == 2 else None

        def worker():
            result = solve_cbs(
                self.grid, agents_snap,
                max_time=300, agent_radius=self.agent_radius,
                stop_event=self._stop_event,
                max_nodes=self.max_nodes,
                progress=self._cont_progress,
                horizon=rhcr_horizon,
            )
            self._cont_cbs_res = (result, list(indices))
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
        root.destroy()
        if not n:
            return

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
            self.agents.append({"start": starts[i], "goal": goals[i]})
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
            defaults = [
                ("Columns (cells)", self.grid.width),
                ("Rows    (cells)", self.grid.height),
                ("Cell size (px)",  self.cell),
            ]
            for row_i, (label, default) in enumerate(defaults):
                tk.Label(dlg, text=label, anchor="w", width=18).grid(
                    row=row_i, column=0, padx=10, pady=6, sticky="w")
                var = tk.StringVar(value=str(default))
                tk.Entry(dlg, textvariable=var, width=8).grid(
                    row=row_i, column=1, padx=10, pady=6)
                fields[label] = var

            result = {}

            def ok():
                try:
                    result["cols"] = max(4,  min(200, int(fields["Columns (cells)"].get())))
                    result["rows"] = max(4,  min(200, int(fields["Rows    (cells)"].get())))
                    result["cell"] = max(CELL_MIN, min(CELL_MAX, int(fields["Cell size (px)"].get())))
                    dlg.destroy()
                except ValueError:
                    pass

            def cancel():
                dlg.destroy()

            tk.Button(dlg, text="OK",     command=ok,     width=8).grid(row=3, column=0, pady=10)
            tk.Button(dlg, text="Cancel", command=cancel, width=8).grid(row=3, column=1, pady=10)
            dlg.bind("<Return>", lambda _: ok())
            dlg.bind("<Escape>", lambda _: cancel())

            root.wait_window(dlg)
            root.destroy()

            if result:
                self.grid   = Grid.empty(result["cols"], result["rows"])
                self.cell   = result["cell"]
                self.agents = []
                self.sel    = 0
                self.cam_x  = 0
                self.cam_y  = 0
                self._reset_sim()
                self._rebuild_btns()
                sw = self.grid.width  * self.cell + SIDEBAR_W
                sh = max(self.grid.height * self.cell, 520)
                self.screen = pygame.display.set_mode((sw, sh), pygame.RESIZABLE)
                self._show(f"New map: {result['cols']}x{result['rows']}, cell={result['cell']}px")
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
            self.sim_t    += dt * self.sim_speed
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
                self._astar_results = {}
                if self._astar_pending:
                    self._launch_astar_batch()

            # Pick up CBS result from worker
            if self._cont_cbs_res is not None:
                result, indices = self._cont_cbs_res
                self._cont_cbs_res = None
                if result:
                    frac = getattr(self, '_cont_cbs_frac', {})
                    for j, idx in enumerate(indices):
                        if idx not in self._cont_agents or j not in result:
                            continue
                        # Preserve fractional local_t so robot doesn't snap back visually
                        self._cont_agents[idx]["path"]    = result[j]
                        self._cont_agents[idx]["local_t"] = frac.get(idx, 0.0)
                        self._rhcr_arrived.discard(idx)
                else:
                    reason = self._cont_progress.get('terminated', 'unknown')
                    nodes  = self._cont_progress.get('nodes', 0)
                    self._show(f"CBS failed [{reason} nodes={nodes}] — holding position", secs=4)
                if self._cont_cbs_pend:
                    self._cont_cbs_pend = False

            import random, math
            min_dist = self.agent_radius * 2 + 0.5
            safe     = self._safe_cache or set()

            if self._cont_replan_mode == 1:
                # ── CBS round-trip: ALL arrive → assign new goals → replan ────
                for idx, ca in self._cont_agents.items():
                    path = ca["path"]
                    at_goal = (int(ca["local_t"]) >= len(path) - 1
                               and path[-1] == ca["goal"])
                    if not at_goal:
                        ca["local_t"] += dt * self.sim_speed

                all_at_goal = all(
                    int(ca["local_t"]) >= len(ca["path"]) - 1
                    and ca["path"][-1] == ca["goal"]
                    for ca in self._cont_agents.values()
                )
                if all_at_goal and not self._cont_cbs_run:
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
                    self._cont_replan_cbs_all()

            elif self._cont_replan_mode == 2:
                # ── RHCR: freeze during CBS, then replan every W steps ────────
                if not self._cont_cbs_run:
                    self._rhcr_global_t += dt * self.sim_speed

                    # Advance agents; clamp at end of path
                    for idx, ca in self._cont_agents.items():
                        path = ca["path"]
                        if int(ca["local_t"]) < len(path) - 1:
                            ca["local_t"] += dt * self.sim_speed
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
                            new_goal = self._pick_new_goal(idx)
                            if new_goal:
                                ca["goal"] = new_goal

                    # H steps elapsed → launch CBS for next W-step window
                    if self._rhcr_global_t - self._rhcr_last_t >= self._rhcr_h:
                        self._rhcr_last_t = self._rhcr_global_t
                        # Snap local_t to integer so CBS result has no visual jump
                        for ca2 in self._cont_agents.values():
                            ca2["local_t"] = float(int(ca2["local_t"]))
                        self._cont_replan_cbs_all()

            else:
                # ── A* mode: arrived OR stuck → assign new goal → replan ──────
                for idx, ca in self._cont_agents.items():
                    path = ca["path"]
                    at_goal = (int(ca["local_t"]) >= len(path) - 1
                               and path[-1] == ca["goal"])
                    if not at_goal:
                        ca["local_t"] += dt * self.sim_speed

                for idx, ca in self._cont_agents.items():
                    if idx in self._astar_pending:
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

                    self._astar_pending[idx] = ca["goal"]

                self._launch_astar_batch()

        if self.msg_ticks > 0:
            self.msg_ticks -= 1

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
        if self._solving and self._show_progress:
            self._draw_cbs_progress()
        if self._cont_mode and self._cont_cbs_run and self._show_progress:
            self._draw_cbs_progress(progress=self._cont_progress)
        if self._cont_mode:
            self._draw_cont_agents()
        else:
            self._draw_paths()
            self._draw_agents()
        self._draw_sidebar()
        self._draw_stat_bar()
        if self.msg and self.msg_ticks > 0:
            self._draw_msg()
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

            # Remaining path (dim line)
            for j in range(step, len(path) - 1):
                x0 = int(path[j][0]   * cell + cell/2 + self.cam_x)
                y0 = int(path[j][1]   * cell + cell/2 + self.cam_y)
                x1 = int(path[j+1][0] * cell + cell/2 + self.cam_x)
                y1 = int(path[j+1][1] * cell + cell/2 + self.cam_y)
                pygame.draw.line(self.screen, dim, (x0, y0), (x1, y1), 1)

            # Goal marker
            gc, gr = ca["goal"]
            gx = int(gc * cell + cell/2 + self.cam_x)
            gy = int(gr * cell + cell/2 + self.cam_y)
            s  = max(3, cell // 3)
            pygame.draw.rect(self.screen, color, (gx-s, gy-s, s*2, s*2), 2)

            # Interpolated agent position
            if step < len(path) - 1:
                c0, r0 = path[step]
                c1, r1 = path[step + 1]
                sx = (c0 + (c1 - c0) * frac) * cell + cell/2 + self.cam_x
                sy = (r0 + (r1 - r0) * frac) * cell + cell/2 + self.cam_y
            else:
                c0, r0 = path[step]
                sx = c0 * cell + cell/2 + self.cam_x
                sy = r0 * cell + cell/2 + self.cam_y

            pygame.draw.circle(self.screen, color, (int(sx), int(sy)), radius)
            pygame.draw.circle(self.screen, (255, 255, 255), (int(sx), int(sy)), radius, 1)

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

    def _draw_grid(self):
        surf  = self.screen
        cell  = self.cell
        cam_x = self.cam_x
        cam_y = self.cam_y
        ga_w  = self._grid_area_w()
        ga_h  = self.screen.get_height()

        # Only draw cells visible on screen
        c0 = max(0, int(-cam_x / cell))
        r0 = max(0, int(-cam_y / cell))
        c1 = min(self.grid.width,  int((ga_w - cam_x) / cell) + 1)
        r1 = min(self.grid.height, int((ga_h - cam_y) / cell) + 1)

        for r in range(r0, r1):
            for c in range(c0, c1):
                x = int(c * cell + cam_x)
                y = int(r * cell + cam_y)
                color = OBS_CELL if self.grid.is_obstacle(c, r) else FREE_CELL
                pygame.draw.rect(surf, color, (x, y, cell, cell))

        # Grid lines
        for c in range(c0, c1 + 1):
            lx = int(c * cell + cam_x)
            pygame.draw.line(surf, GRID_LINE, (lx, 0), (lx, ga_h))
        for r in range(r0, r1 + 1):
            ly = int(r * cell + cam_y)
            pygame.draw.line(surf, GRID_LINE, (0, ly), (ga_w, ly))

    def _draw_paths(self):
        if not self.paths:
            return
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
                pygame.draw.line(self.screen, dim, (cx0, cy0), (cx1, cy1), max(1, self.cell // 6))

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

        # Buttons
        for b in self._btns:
            b.draw(self.screen, self.font_s)

        # Agent radius control
        ry = self._y_radius
        bar_w = SIDEBAR_W - 20

        # Radius bar
        ry = self._y_radius
        self.screen.blit(self.font_s.render(f"Radius: {self.agent_radius:.1f} cells  [z/x]", True, DIM), (x0+8, ry))
        r_min, r_max = 0.5, 4.0
        filled_r = int(bar_w * (self.agent_radius - r_min) / (r_max - r_min))
        pygame.draw.rect(self.screen, (55, 55, 70),   (x0+8, ry+13, bar_w, 8), border_radius=4)
        pygame.draw.rect(self.screen, (180, 120, 60), (x0+8, ry+13, filled_r, 8), border_radius=4)

        # Speed bar
        sy = self._y_speed
        self.screen.blit(self.font_s.render(f"Speed: {self.sim_speed:.1f}x   [+/-]", True, DIM), (x0+8, sy))
        filled = int(bar_w * (self.sim_speed - 0.5) / 19.5)
        pygame.draw.rect(self.screen, (55, 55, 70), (x0+8, sy+13, bar_w, 8), border_radius=4)
        pygame.draw.rect(self.screen, BTN_ACTIVE,   (x0+8, sy+13, filled, 8), border_radius=4)

        # Timeout row
        ty = sy + 30
        self._draw_adj_row(x0, ty, bar_w, f"Timeout: {self.solve_timeout:.0f}s",
                           '_rect_timeout_minus', '_rect_timeout_plus')

        # Max-nodes row
        self._draw_adj_row(x0, ty+18, bar_w, f"Max nodes: {self.max_nodes}",
                           '_rect_nodes_minus', '_rect_nodes_plus')

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
        self.screen.blit(self.font_s.render(header, True, DIM), (x0+8, ay))
        ay += 14
        self._agent_list_top = ay   # store for scroll hit-test

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
            row_txt = self.font_s.render(f" A{i+1} S{s_s} G{g_s}", True, TEXT if is_sel else DIM)
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

        # SOLVE 버튼 라벨 동적 변경
        self.b_solve.label = "CANCEL [Space]" if self._solving else "SOLVE CBS [Space]"
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
                mode_s = ["A*", "CBS", "RHCR"][self._cont_replan_mode]
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

        parts = [f"FPS:{self._stat_fps:5.1f}", f"frame:{self._stat_frame_ms:5.1f}ms"]

        if self._cont_mode:
            if self._cont_replan_mode in (1, 2) and self._cont_cbs_run:
                nodes = self._cont_progress.get('nodes', 0)
                parts.append(f"CBS nodes:{nodes}")
            elif self._cont_replan_mode == 2:
                time_to_next = max(0.0, self._rhcr_h - (self._rhcr_global_t - self._rhcr_last_t))
                parts.append(f"RHCR W={self._rhcr_window} H={self._rhcr_h} nxt:{time_to_next:.1f}  [[ ]],<.>")
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

        txt = self.font_s.render("  |  ".join(parts), True, (140, 140, 155))
        self.screen.blit(txt, (6, y + 2))

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
                    running = False

                elif ev.type == pygame.KEYDOWN:
                    k = ev.key
                    if   k == pygame.K_SPACE:                    self._solve_or_cancel()
                    elif k == pygame.K_r:                        self._reset_sim()
                    elif k == pygame.K_ESCAPE:
                        self.edit_sub = SUB_NONE
                        self._refresh_btn_state()
                    elif k == pygame.K_1:                        self._set_mode(MODE_OBS)
                    elif k == pygame.K_2:                        self._set_mode(MODE_AGNT)
                    elif k == pygame.K_3:                        self._set_mode(MODE_ERASE)
                    elif k == pygame.K_s and self.mode==MODE_AGNT: self._toggle_start()
                    elif k == pygame.K_g and self.mode==MODE_AGNT: self._toggle_goal()
                    elif k in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
                        self.sim_speed = min(20.0, self.sim_speed + 0.5)
                    elif k in (pygame.K_MINUS, pygame.K_KP_MINUS):
                        self.sim_speed = max(0.5, self.sim_speed - 0.5)
                    elif k == pygame.K_x:
                        self.agent_radius = round(min(4.0, self.agent_radius + 0.5), 1)
                    elif k == pygame.K_z:
                        self.agent_radius = round(max(0.5, self.agent_radius - 0.5), 1)
                    elif k == pygame.K_a:
                        self._start_auto(0)
                    elif k == pygame.K_b:
                        self._start_auto(1)
                    elif k == pygame.K_h:
                        self._start_auto(2)
                    elif k == pygame.K_TAB:
                        self._show_progress = not self._show_progress
                    elif k == pygame.K_LEFTBRACKET:
                        self._rhcr_window = max(self._rhcr_h, self._rhcr_window - 5)
                        self._show(f"RHCR W={self._rhcr_window} H={self._rhcr_h}")
                    elif k == pygame.K_RIGHTBRACKET:
                        self._rhcr_window = min(200, self._rhcr_window + 5)
                        self._show(f"RHCR W={self._rhcr_window} H={self._rhcr_h}")
                    elif k == pygame.K_COMMA:
                        self._rhcr_h = max(1, self._rhcr_h - 1)
                        self._rhcr_window = max(self._rhcr_window, self._rhcr_h)
                        self._show(f"RHCR W={self._rhcr_window} H={self._rhcr_h}")
                    elif k == pygame.K_PERIOD:
                        self._rhcr_h = min(self._rhcr_window, self._rhcr_h + 1)
                        self._show(f"RHCR W={self._rhcr_window} H={self._rhcr_h}")
                    # F1-F8 to select agent
                    elif pygame.K_F1 <= k <= pygame.K_F8:
                        idx = k - pygame.K_F1
                        if idx < len(self.agents):
                            self.sel = idx

                elif ev.type == pygame.MOUSEBUTTONDOWN:
                    # Timeout mini-buttons
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
                    # Sidebar buttons
                    handled = any(b.on_event(ev) for b in self._btns)
                    if not handled:
                        if ev.button in (1, 3):
                            self._on_grid_click(ev.pos[0], ev.pos[1], ev.button)
                        elif ev.button == 2:
                            self._panning  = True
                            self._pan_orig = ev.pos

                elif ev.type == pygame.MOUSEBUTTONUP:
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
                    else:                                  # scroll agent list
                        top = getattr(self, '_agent_list_top', 9999)
                        if my >= top:
                            self._agent_list_scroll = max(
                                0, min(len(self.agents) - 1,
                                       self._agent_list_scroll - ev.y)
                            )

                elif ev.type == pygame.VIDEORESIZE:
                    self._screen_w = ev.w
                    self._screen_h = ev.h
                    self._rebuild_btns()

            self.update(dt)
            self.draw()

        pygame.quit()
