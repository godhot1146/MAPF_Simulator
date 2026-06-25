import os
import numpy as np

try:
    import yaml
    HAS_YAML = True
except ImportError:
    HAS_YAML = False


class Grid:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.obstacles = set()
        self.forbidden = set()
        self.speed_zones = {}  # {(c, r): max_speed_tpc} — minimum tpc required in zone
        self.resolution = 0.05  # meters per cell

    def in_bounds(self, c, r):
        return 0 <= c < self.width and 0 <= r < self.height

    def is_obstacle(self, c, r):
        if not self.in_bounds(c, r):
            return True
        return (c, r) in self.obstacles

    def is_free(self, c, r):
        return not self.is_obstacle(c, r) and (c, r) not in self.forbidden

    def set_obstacle(self, c, r, val):
        if not self.in_bounds(c, r):
            return
        if val:
            self.obstacles.add((c, r))
        else:
            self.obstacles.discard((c, r))

    def neighbors4(self, c, r):
        result = []
        for dc, dr in ((0, 1), (0, -1), (1, 0), (-1, 0)):
            nc, nr = c + dc, r + dr
            if self.is_free(nc, nr):
                result.append((nc, nr))
        return result

    @classmethod
    def empty(cls, width, height):
        return cls(width, height)

    @classmethod
    def from_pgm_yaml(cls, yaml_path):
        if not HAS_YAML:
            raise ImportError("Install pyyaml: pip install pyyaml")
        with open(yaml_path, "r") as f:
            meta = yaml.safe_load(f)

        img_file = meta.get("image", "")
        pgm_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), img_file)
        occupied_thresh = meta.get("occupied_thresh", 0.65)
        negate = meta.get("negate", 0)

        pixels = load_pgm(pgm_path)
        h, w = pixels.shape
        grid = cls(w, h)

        for r in range(h):
            for c in range(w):
                p = pixels[r, c] / 255.0
                if negate:
                    p = 1.0 - p
                # p = "freeness". obstacle when occupancy probability > occupied_thresh
                if p < (1.0 - occupied_thresh):
                    grid.obstacles.add((c, r))

        grid.resolution = meta.get("resolution", 0.05)
        for entry in meta.get("forbidden", []):
            grid.forbidden.add((int(entry[0]), int(entry[1])))
        for entry in meta.get("speed_zones", []):
            grid.speed_zones[(int(entry[0]), int(entry[1]))] = int(entry[2])

        return grid, meta

    def save_pgm_yaml(self, pgm_path, yaml_path=None, resolution=0.05):
        if yaml_path is None:
            yaml_path = os.path.splitext(pgm_path)[0] + ".yaml"

        pixels = np.full((self.height, self.width), 255, dtype=np.uint8)
        for c, r in self.obstacles:
            if self.in_bounds(c, r):
                pixels[r, c] = 0

        save_pgm(pgm_path, pixels)

        if HAS_YAML:
            meta = {
                "image": os.path.basename(pgm_path),
                "resolution": self.resolution,
                "origin": [0.0, 0.0, 0.0],
                "occupied_thresh": 0.65,
                "free_thresh": 0.196,
                "negate": 0,
            }
            if self.forbidden:
                meta["forbidden"] = [[c, r] for c, r in sorted(self.forbidden)]
            if self.speed_zones:
                meta["speed_zones"] = [[c, r, tpc] for (c, r), tpc in sorted(self.speed_zones.items())]
            with open(yaml_path, "w") as f:
                yaml.dump(meta, f, default_flow_style=False)


def load_pgm(path):
    with open(path, "rb") as f:
        magic = f.readline().decode("latin-1").strip()
        line = f.readline().decode("latin-1").strip()
        while line.startswith("#"):
            line = f.readline().decode("latin-1").strip()
        parts = line.split()
        w, h = int(parts[0]), int(parts[1])
        maxval = int(f.readline().decode("latin-1").strip())

        if magic == "P5":
            raw = f.read(w * h)
            pixels = np.frombuffer(raw, dtype=np.uint8).reshape((h, w))
        elif magic == "P2":
            vals = []
            for line in f:
                vals.extend(line.decode("latin-1").split())
            pixels = np.array([int(v) for v in vals], dtype=np.uint8).reshape((h, w))
        else:
            raise ValueError(f"Unsupported PGM format: {magic}")

        if maxval != 255:
            pixels = (pixels.astype(np.float32) / maxval * 255).astype(np.uint8)

    return pixels


def save_pgm(path, pixels):
    h, w = pixels.shape
    with open(path, "wb") as f:
        f.write(f"P5\n{w} {h}\n255\n".encode("ascii"))
        f.write(pixels.tobytes())
