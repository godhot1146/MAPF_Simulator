import os
import argparse
from simulator import MAPFApp, CELL_DEFAULT, CELL_MIN, CELL_MAX

if __name__ == "__main__":
    os.makedirs("maps", exist_ok=True)

    parser = argparse.ArgumentParser(description="MAPF Simulator (CBS)")
    parser.add_argument("--cols", type=int, default=40,          help="Grid width in cells  (default: 40)")
    parser.add_argument("--rows", type=int, default=30,          help="Grid height in cells (default: 30)")
    parser.add_argument("--cell", type=int, default=CELL_DEFAULT, help=f"Cell size in pixels  (default: {CELL_DEFAULT}, range: {CELL_MIN}-{CELL_MAX})")
    args = parser.parse_args()

    cols = max(4,        min(200,      args.cols))
    rows = max(4,        min(200,      args.rows))
    cell = max(CELL_MIN, min(CELL_MAX, args.cell))

    app = MAPFApp(cols=cols, rows=rows, cell=cell)
    app.run()
