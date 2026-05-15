# MAPF Simulator

Multi-Agent Path Finding simulator with CBS, Auto A*, and RHCR.

- **Algorithms**: CBS (manual solve) · Auto A* · Auto CBS · Auto RHCR
- **Map format**: PGM + YAML (ROS map_server compatible)
- **Collision model**: circular agent body — obstacle clearance + inter-agent distance
- **Max agents**: 99

---

## Quick Start

```bash
pip install -r requirements.txt
python make_maps.py   # generate example maps (first time only)
python main.py
```

### CLI options

```bash
python main.py --cols 200 --rows 100 --cell 8
```

| Flag | Default | Description |
|---|---|---|
| `--cols` | 40 | Grid width in cells |
| `--rows` | 30 | Grid height in cells |
| `--cell` | 8 | Cell size in pixels (8–40) |

Last used map and settings are restored automatically via `config.json`.

---

## Project Structure

```
MAPF_Simulator/
├── main.py           Entry point (CLI args + config restore)
├── grid.py           Grid class + PGM/YAML I/O
├── astar.py          Space-time A* (CBS constraints, clearance, partial path)
├── cbs.py            CBS high-level planner (horizon-limited conflict detection)
├── simulator.py      Pygame UI — editor + all simulation modes
├── make_maps.py      Example map generator
├── requirements.txt
└── maps/
    ├── empty_40x30.pgm / .yaml
    ├── warehouse_40x30.pgm / .yaml
    ├── warehouse_200x100.pgm / .yaml
    ├── random_32x24.pgm / .yaml
    ├── benchmark_33x33.pgm / .yaml
    └── benchmark_65x65.pgm / .yaml
```

---

## Controls

### Edit Modes

| Key | Mode |
|---|---|
| `1` | Draw obstacles |
| `3` | Erase obstacles |
| `2` | Agent edit |
| `Esc` | Cancel action |

### Obstacle / Erase (mode `1` / `3`)

| Input | Action |
|---|---|
| Left-click / drag | Place obstacle |
| Right-click / drag | Erase obstacle |

### Agent Edit (mode `2`)

| Input | Action |
|---|---|
| `+Agent` button | Add agent (max 99) |
| `-Agent` button | Delete selected agent |
| `s` → click | Set start position |
| `g` → click | Set goal position |
| `F1`–`F8` | Select agent by index |
| `Random Agents` button | Auto-place N agents (spread-sampled, no overlap) |

### Simulation (manual CBS)

| Key / Button | Action |
|---|---|
| `Space` / `SOLVE CBS` | Run CBS solver (background thread) |
| `Space` / `CANCEL` | Cancel ongoing solve |
| `r` / `Reset Sim` | Reset simulation |
| `+` / `-` | Playback speed (0.5×–20×) |
| `Tab` | Toggle live CBS visualisation ON/OFF |

### Auto Mode (Lifelong MAPF)

| Key | Button | Mode |
|---|---|---|
| `a` | `Auto: A*` | Individual A* replan on arrival |
| `b` | `Auto: CBS` | CBS round-trip (all arrive → replan) |
| `h` | `Auto: RHCR` | Rolling Horizon CBS (every H steps) |

Pressing the same button again stops the mode. Pressing a different button switches mode.

#### RHCR Parameters

| Key | Action |
|---|---|
| `[` / `]` | W (planning horizon) −5 / +5 |
| `,` / `.` | H (execution horizon) −1 / +1 |

Stat bar shows: `RHCR W=12 H=5 nxt:3.2steps`

### View

| Input | Action |
|---|---|
| Mouse wheel | Zoom in/out (8–40 px/cell) |
| Middle-button drag | Pan |

### Agent Body

| Key | Action |
|---|---|
| `z` | Radius −0.5 cells |
| `x` | Radius +0.5 cells |

### File / Map

| Button | Action |
|---|---|
| `Save Map` | Save grid as PGM+YAML |
| `Load Map` | Load PGM+YAML map |
| `New Map...` | Set cols × rows × cell size |
| `Clear Agents` | Remove all agents |
| `Clear All` | Clear map and agents |

### Solver Settings (sidebar)

| Control | Range | Step |
|---|---|---|
| Timeout | 5–300 s | 10 s |
| Max nodes | 500–100 000 | 500 |

---

## Auto Modes Explained

### Auto: A*

Each robot replans individually when it arrives at its goal. Uses soft body constraints from other robots' current paths. Fast, but collision-free not strictly guaranteed.

### Auto: CBS

All robots move toward their goals. When **all** arrive, new goals are assigned and a single CBS solve runs. Collision-free within each round. Best for accurate simulation with fewer agents.

### Auto: RHCR (Rolling Horizon Collision Resolution)

| Parameter | Meaning |
|---|---|
| W (window) | Steps CBS looks ahead for conflicts |
| H (horizon) | Steps executed before replanning (H ≤ W) |

Flow: **CBS runs** (W-step conflict window) → **H steps executed** → **CBS runs again** → repeat.

- Agents freeze during CBS computation (no teleport)
- CBS plans full paths but only resolves conflicts within W steps
- Theoretically guarantees collision-free execution within each window
- Recommended: W=12, H=5 for 33×33 maps with 10–12 agents

---

## Agent Body & Collision

Each agent has a configurable **radius** (cells, default 0.5).

- **Obstacle clearance**: A* only visits cells where all cells within `radius` are free.
- **Inter-agent collision**: two agents conflict when distance < `2 × radius`.
- **Visual**: agent circle drawn at `radius × cell_size` pixels.

---

## CBS Solver Details

| Feature | Description |
|---|---|
| Low-level planner | Space-time A* — vertex + edge constraints, partial path support |
| Duplicate detection | Path-combination memoization |
| Conflict types | Vertex (distance-based) + edge (swap) |
| RHCR horizon | Conflict detection limited to W steps via `horizon` parameter |
| Termination | `[PROVEN]` no solution / `[NODE LIMIT]` / `[TIMEOUT]` / `Cancelled` |

### Solver Messages

| Message | Meaning |
|---|---|
| `Solution found!` | Optimal conflict-free paths found |
| `[PROVEN] No solution exists` | Search space exhausted — impossible configuration |
| `[NODE LIMIT N]` | Node cap reached — increase limit or reduce agents |
| `[TIMEOUT Xs]` | Time cap reached |
| `CBS failed [no_solution]` | Auto-mode replan failed — A* fallback used |

---

## Maps

| File | Size | Description |
|---|---|---|
| `empty_40x30` | 40×30 | Open grid with border walls |
| `warehouse_40x30` | 40×30 | Vertical shelf layout |
| `warehouse_200x100` | 200×100 | Large warehouse with cross-aisles |
| `random_32x24` | 32×24 | 20% random obstacles |
| `benchmark_33x33` | 33×33 | MAPF benchmark warehouse (horizontal shelves) |
| `benchmark_65x65` | 65×65 | Large MAPF benchmark warehouse |

Regenerate: `python make_maps.py`

---

## Throughput Metric

Stat bar shows `throughput: N tasks/min` — a 60-second sliding window count of goal arrivals. Use this to compare algorithm efficiency.
