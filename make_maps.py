"""
Run once to create example PGM+YAML maps in the maps/ directory.
  python make_maps.py
"""
import os
import numpy as np
import sys

sys.path.insert(0, os.path.dirname(__file__))
from grid import Grid, save_pgm

os.makedirs("maps", exist_ok=True)


def write_map(name, pixels):
    pgm  = f"maps/{name}.pgm"
    yaml = f"maps/{name}.yaml"
    save_pgm(pgm, pixels)
    try:
        import yaml as _yaml
        meta = {
            "image": f"{name}.pgm",
            "resolution": 0.05,
            "origin": [0.0, 0.0, 0.0],
            "occupied_thresh": 0.65,
            "free_thresh": 0.196,
            "negate": 0,
        }
        with open(yaml, "w") as f:
            _yaml.dump(meta, f, default_flow_style=False)
        print(f"  Created {pgm} + {yaml}")
    except ImportError:
        print(f"  Created {pgm}  (pyyaml not installed – no .yaml)")


# ── 1. Empty 40×30 ────────────────────────────────────────────────────────────
def make_empty(w=40, h=30):
    p = np.full((h, w), 255, dtype=np.uint8)
    # border walls
    p[0, :]  = 0
    p[-1, :] = 0
    p[:, 0]  = 0
    p[:, -1] = 0
    return p

write_map("empty_40x30", make_empty(40, 30))


# ── 2. Warehouse-like 40×30 ───────────────────────────────────────────────────
def make_warehouse(w=40, h=30):
    p = make_empty(w, h)
    # Vertical shelves
    for shelf_x in range(5, w-5, 6):
        for row in range(3, h-3):
            if (row // 3) % 2 == 0:          # gap every 3 rows
                p[row, shelf_x] = 0
    return p

write_map("warehouse_40x30", make_warehouse(40, 30))


# ── 3. Maze 32×24 ─────────────────────────────────────────────────────────────
def make_maze(w=32, h=24, seed=42):
    rng = np.random.default_rng(seed)
    p   = np.full((h, w), 255, dtype=np.uint8)
    # border
    p[0, :] = p[-1, :] = p[:, 0] = p[:, -1] = 0
    # random interior walls (~20 % density)
    interior = rng.random((h-2, w-2)) < 0.20
    p[1:h-1, 1:w-1][interior] = 0
    return p

write_map("random_32x24", make_maze(32, 24))


# ── 4. Large Warehouse 200×100 ───────────────────────────────────────────────
def make_warehouse_large(w=200, h=100):
    """
    Warehouse layout:
    - Border walls
    - Horizontal shelves (2 cells tall) with cross-aisles every 10 cols
    - Shelf rows every 10 rows, leaving 8-row aisles between them
    - End-caps left open for entry/exit
    """
    p = np.full((h, w), 255, dtype=np.uint8)

    # Border walls
    p[0, :] = p[-1, :] = p[:, 0] = p[:, -1] = 0

    shelf_height  = 2   # obstacle rows per shelf
    aisle_gap     = 8   # free rows between shelves
    period        = shelf_height + aisle_gap   # 10

    col_aisle_every = 10  # vertical gap cut through every shelf every N cols
    endcap          = 4   # free cols at left/right ends of each shelf

    shelf_start_row = period  # first shelf at row 10

    for shelf_row in range(shelf_start_row, h - period, period):
        for sr in range(shelf_height):
            row = shelf_row + sr
            if row >= h - 1:
                break
            for col in range(1, w - 1):
                # leave end-caps open
                if col < endcap or col >= w - endcap:
                    continue
                # leave vertical aisles (every col_aisle_every columns)
                if col % col_aisle_every == 0:
                    continue
                p[row, col] = 0

    return p

write_map("warehouse_200x100", make_warehouse_large(200, 100))

print("Done - maps/ folder ready.")
