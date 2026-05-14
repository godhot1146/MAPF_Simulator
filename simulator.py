import threading
import time
import os
import pygame

from grid import Grid
from cbs import solve_cbs

# ── Palette ────────────────────────────────────────────────────────────────────
BG            = (28,  28,  32)
GRID_LINE     = (50,  50,  56)
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

AGENT_COLORS = [
    (230,  75,  75),
    ( 75, 200,  75),
    ( 75, 120, 230),
    (230, 185,  55),
    (185,  75, 220),
    ( 55, 205, 205),
    (230, 130,  55),
    (145, 225,  75),
]

SIDEBAR_W   = 225
CELL_DEFAULT      =  8
SOLVE_TIMEOUT_DEFAULT = 30.0
CELL_MIN     =  8
CELL_MAX     = 40

MODE_OBS  = "obstacle"
MODE_AGNT = "agent"

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
    def __init__(self, cols=40, rows=30, cell=CELL_DEFAULT):
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
        self.sim_speed    = 5.0         # steps / second
        self.agent_radius = 1.5         # physical radius in cells

        self._solving       = False
        self._sol_result    = None
        self._sol_error     = ""
        self._stop_event    = threading.Event()
        self._solve_start   = 0.0
        self.solve_timeout  = SOLVE_TIMEOUT_DEFAULT   # adjustable
        self.max_nodes      = 5000                    # CBS node expansion limit
        self._cbs_progress  = {}
        self._show_progress = True     # toggle with Tab

        self._drag_painting = False
        self._drag_val      = True

        self.msg       = ""
        self.msg_ticks = 0

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

        self.b_obs   = B((x0, y, w,      h), "Obstacle [1]",  lambda: self._set_mode(MODE_OBS),  True); y += h+g
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
        self.b_solve  = B((x0, y, w, h), "SOLVE  [Space]", self._solve_or_cancel);          y += h+g
        self.b_rsim  = B((x0, y, w, h), "Reset Sim  [r]", self._reset_sim);                y += h+g*2
        self.b_save  = B((x0,       y, hw, h), "Save Map", self._save_map)
        self.b_load  = B((x0+hw+4,  y, hw, h), "Load Map", self._load_map);                y += h+g
        self.b_new   = B((x0, y, w, h), "New Map...",     self._new_map);                  y += h+g
        self.b_clag  = B((x0, y, w, h), "Clear Agents",   self._clear_agents);             y += h+g
        self.b_call  = B((x0, y, w, h), "Clear All",      self._clear_all)

        self._btns = [
            self.b_obs, self.b_agnt,
            self.b_add, self.b_del,
            self.b_start, self.b_goal,
            self.b_solve, self.b_rsim,
            self.b_save, self.b_load,
            self.b_new, self.b_clag, self.b_call,
        ]
        self._refresh_btn_state()

    def _refresh_btn_state(self):
        self.b_obs.active   = (self.mode == MODE_OBS)
        self.b_agnt.active  = (self.mode == MODE_AGNT)
        self.b_start.active = (self.edit_sub == SUB_START)
        self.b_goal.active  = (self.edit_sub == SUB_GOAL)

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
        if len(self.agents) >= 8:
            self._show("Max 8 agents.")
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
        self.sim_run    = False
        self.sim_t      = 0.0
        self.sim_step   = 0
        self.paths      = {}
        self._sol_result = None

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

        if self.msg_ticks > 0:
            self.msg_ticks -= 1

    # ── Draw ──────────────────────────────────────────────────────────────────
    def draw(self):
        self.screen.fill(BG)
        self._draw_grid()
        if self._solving and self._show_progress:
            self._draw_cbs_progress()
        self._draw_paths()
        self._draw_agents()
        self._draw_sidebar()
        if self.msg and self.msg_ticks > 0:
            self._draw_msg()
        pygame.display.flip()

    def _draw_cbs_progress(self):
        prog = self._cbs_progress
        if not prog:
            return
        cell  = self.cell
        ga_w  = self._grid_area_w()
        ga_h  = self.screen.get_height()

        # ── Candidate paths (dim, thin) ───────────────────────────────────────
        cand_paths = prog.get('paths', {})
        for idx, path in cand_paths.items():
            color = AGENT_COLORS[self._valid_indices[idx] % len(AGENT_COLORS)] if hasattr(self, '_valid_indices') and idx < len(self._valid_indices) else AGENT_COLORS[idx % len(AGENT_COLORS)]
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
            color = AGENT_COLORS[i % len(AGENT_COLORS)]
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
            color  = AGENT_COLORS[i % len(AGENT_COLORS)]
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

        # Agent list
        ay = self.b_call.rect.bottom + 12
        self.screen.blit(self.font_s.render("Agents:", True, DIM), (x0+8, ay))
        ay += 15
        for i, a in enumerate(self.agents):
            color  = AGENT_COLORS[i % len(AGENT_COLORS)]
            is_sel = (i == self.sel)
            if is_sel:
                pygame.draw.rect(self.screen, (60, 65, 92),
                                 (x0+6, ay-1, SIDEBAR_W-12, 16), border_radius=3)
            pygame.draw.rect(self.screen, color, (x0+8, ay+3, 9, 9))
            s_s = f"({a['start'][0]},{a['start'][1]})" if a["start"] else "?"
            g_s = f"({a['goal'][0]},{a['goal'][1]})"   if a["goal"]  else "?"
            row = self.font_s.render(f" A{i+1} S{s_s} G{g_s}", True, TEXT if is_sel else DIM)
            self.screen.blit(row, (x0+20, ay))
            ay += 16

        # SOLVE 버튼 라벨 동적 변경
        self.b_solve.label = "CANCEL [Space]" if self._solving else "SOLVE  [Space]"
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
        m_str   = f"Mode: {self.mode}{sub_str}"
        self.screen.blit(self.font_s.render(m_str,                True, (140, 140, 200)), (x0+8, sh-28))
        self.screen.blit(self.font_s.render("MMB=pan  Wheel=zoom", True, DIM),             (x0+8, sh-14))

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
                    elif k == pygame.K_TAB:
                        self._show_progress = not self._show_progress
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
                    if mx < self._grid_area_w():          # zoom only on grid area
                        old = self.cell
                        self.cell = max(CELL_MIN, min(CELL_MAX, self.cell + ev.y))
                        # Zoom towards mouse
                        gx = (mx - self.cam_x) / old
                        gy = (my - self.cam_y) / old
                        self.cam_x = mx - gx * self.cell
                        self.cam_y = my - gy * self.cell
                        self._clamp_cam()

                elif ev.type == pygame.VIDEORESIZE:
                    self._screen_w = ev.w
                    self._screen_h = ev.h
                    self._rebuild_btns()

            self.update(dt)
            self.draw()

        pygame.quit()
