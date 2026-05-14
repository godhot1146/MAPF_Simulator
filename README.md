# MAPF Simulator (CBS)

Multi-Agent Path Finding simulator with physical agent bodies.

- **Algorithm**: Conflict-Based Search (CBS) with space-time A*
- **Map format**: PGM + YAML (ROS map_server compatible)
- **Collision model**: circular agent body — obstacle clearance + inter-agent distance

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

---

## Project Structure

```
MAPF_Simulator/
├── main.py           Entry point (CLI args)
├── grid.py           Grid class + PGM/YAML I/O
├── astar.py          Space-time A* with CBS constraints & clearance
├── cbs.py            CBS high-level planner (duplicate-free)
├── simulator.py      Pygame UI — editor + simulator
├── make_maps.py      Example map generator
├── requirements.txt
└── maps/
    ├── empty_40x30.pgm / .yaml
    ├── warehouse_40x30.pgm / .yaml
    ├── warehouse_200x100.pgm / .yaml
    └── random_32x24.pgm / .yaml
```

---

## PGM + YAML Map Format

```yaml
image: map.pgm        # PGM filename (same directory)
resolution: 0.05      # metres/pixel
origin: [0.0, 0.0, 0.0]
occupied_thresh: 0.65
free_thresh: 0.196
negate: 0
```

PGM convention: white (255) = free, black (0) = obstacle.

---

## Controls

### Mode

| Key | Action |
|---|---|
| `1` | Obstacle edit mode |
| `2` | Agent edit mode |
| `Esc` | Cancel current action |

### Obstacle Edit (mode `1`)

| Input | Action |
|---|---|
| Left-click / drag | Place obstacle |
| Right-click / drag | Erase obstacle |

### Agent Edit (mode `2`)

| Input | Action |
|---|---|
| `+Agent` button | Add agent (max 8) |
| `-Agent` button | Delete selected agent |
| `s` or `SetStart` → click grid | Set start position |
| `g` or `SetGoal` → click grid | Set goal position |
| `F1`–`F8` | Select agent by index |
| Click agent marker | Select that agent |

### Simulation

| Key / Button | Action |
|---|---|
| `Space` / `SOLVE` | Run CBS solver (background thread) |
| `Space` / `CANCEL` | Cancel ongoing solve |
| `r` / `Reset Sim` | Reset simulation |
| `+` / `-` | Playback speed (0.5×–20×) |

### View

| Input | Action |
|---|---|
| Mouse wheel | Zoom in/out (8–40 px/cell) |
| Middle-button drag | Pan |
| `Tab` | Toggle live CBS visualisation ON/OFF |

### Agent Body

| Key | Action |
|---|---|
| `z` | Decrease agent radius −0.5 cells |
| `x` | Increase agent radius +0.5 cells |

### File / Map

| Button | Action |
|---|---|
| `Save Map` | Save current grid as PGM+YAML |
| `Load Map` | Load PGM+YAML map |
| `New Map...` | Dialog to set cols × rows × cell size |
| `Clear Agents` | Remove all agents |
| `Clear All` | Clear map and agents |

### Solver Settings (sidebar sliders)

| Control | Range | Step |
|---|---|---|
| Timeout | 5–300 s | 10 s |
| Max nodes | 500–100 000 | 500 |

---

## Agent Body & Collision

Each agent has a configurable **radius** (in cells, default 1.5).

- **Obstacle clearance**: A* only visits cells where all cells within `radius` are free.
- **Inter-agent collision**: two agents conflict when their Euclidean distance < `2 × radius`.
- **Visual**: the agent circle is drawn at `radius × cell_size` pixels.

Minimum corridor width for two agents to pass: `> 4 × radius` cells.

---

## CBS Solver Details

| Feature | Description |
|---|---|
| Low-level planner | Space-time A* with vertex + edge constraints |
| Duplicate detection | Path-combination memoization — same path set is never re-evaluated |
| Conflict types | Vertex conflicts (distance-based) + edge conflicts (swap) |
| Termination | `[PROVEN] No solution` when search space exhausted; `[NODE LIMIT]` / `[TIMEOUT]` otherwise |

### Result messages

| Message | Meaning |
|---|---|
| `Solution found!` | Optimal conflict-free paths found |
| `[PROVEN] No solution exists` | Search space fully exhausted — impossible configuration |
| `[NODE LIMIT N]` | Node cap reached — may be solvable, increase limit or simplify |
| `[TIMEOUT Xs]` | Time cap reached — same as above |
| `Cancelled` | Manually cancelled by user |

---

## Visualisation

| Element | Appearance |
|---|---|
| Agent (static) | Filled circle with color ring |
| Agent (simulating) | Filled circle, smooth interpolation between steps |
| Goal marker | Hollow square in agent color |
| Planned path | Thin line in agent color |
| CBS candidate paths | Dim lines (live, during solve) |
| Conflict marker | Pulsing red ring + `t=N` timestep label |

Agent colors (in order): red, green, blue, yellow, purple, cyan, orange, lime.
