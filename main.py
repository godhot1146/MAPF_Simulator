import os
import json
import argparse
from simulator import MAPFApp, CELL_DEFAULT, CELL_MIN, CELL_MAX

CONFIG_PATH = os.path.join(os.path.dirname(__file__), "config.json")


def load_config():
    if os.path.exists(CONFIG_PATH):
        try:
            with open(CONFIG_PATH, encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            pass
    return {}


if __name__ == "__main__":
    os.makedirs("maps", exist_ok=True)

    parser = argparse.ArgumentParser(description="MAPF Simulator (CBS)")
    parser.add_argument("--cols", type=int, default=None,         help="Grid width in cells")
    parser.add_argument("--rows", type=int, default=None,         help="Grid height in cells")
    parser.add_argument("--cell", type=int, default=CELL_DEFAULT, help=f"Cell size in pixels (default: {CELL_DEFAULT})")
    args = parser.parse_args()

    cfg      = load_config()
    last_map = cfg.get("last_map")

    cols = max(4,        min(200,      args.cols if args.cols else cfg.get("cols", 40)))
    rows = max(4,        min(200,      args.rows if args.rows else cfg.get("rows", 30)))
    # CLI cell overrides config; config overrides default
    cell = max(CELL_MIN, min(CELL_MAX,
               args.cell if args.cell != CELL_DEFAULT else int(cfg.get("cell", CELL_DEFAULT))))

    app = MAPFApp(cols=cols, rows=rows, cell=cell, last_map=last_map, cfg=cfg)
    app.run()
