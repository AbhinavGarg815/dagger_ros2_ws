"""The operator's saved SLAM maps, as files in one folder (no ROS here).

A map <name> is the files slam_toolbox and tools/slam_georef.py leave behind:

    <name>.yaml + <name>.pgm     the picture (map_saver_cli): resolution, origin
    <name>.posegraph + .data     what localization needs (serialize_map)
    <name>_geo.yaml              optional: ties the map to the Earth (slam_georef.py)

For the operator app each map becomes a summary -- size, and when tied to the
Earth the four corners of its picture as lat/lon -- plus a small PNG of the
picture (grey 0 = wall, 255 = free, transparent = never seen), kept in
<folder>/.pictures. Which map is drawn under missions is kept in <folder>/.shown.

The folder is the operator's, outside git (default ~/dagger_maps): the maps in
src/dagger_bringup/maps are the engineer's and are never touched from here.
"""
import os

import numpy as np
import yaml
from PIL import Image

from .geo_map import GeoMap

# Every file a map may have; deleting a map removes these and nothing else.
SUFFIXES = (".yaml", ".pgm", ".posegraph", ".data", "_geo.yaml", "_view.png", "_map.png")
PICTURES = ".pictures"
SHOWN = ".shown"
# map_saver_cli writes walls 0, free 254 and never-seen 205 (trinary).
WALL_BELOW = 100
FREE_ABOVE = 230
WALL_GROW = 2                     # picture only: walls drawn 2 pixels (10 cm) thicker each way


class SavedMap:
    """One map: what the app is told about it (`summary`) and its PNG on disk."""

    def __init__(self, name, width_px, height_px, resolution, corners, placed_error_m, created_ms, picture,
                 geo=()):
        self.name = name
        self.width_px, self.height_px = width_px, height_px
        self.width_m, self.height_m = width_px * resolution, height_px * resolution
        self.corners = corners                  # [(lat, lon)] TL, TR, BR, BL; [] when not placed
        self.placed_error_m = placed_error_m
        self.created_ms = created_ms
        self.picture = picture                  # absolute path of the PNG
        self.version = int(os.path.getmtime(picture))
        self.geo = tuple(geo)                   # (lat0, lon0, yaw_rad, tx, ty) when placed

    @property
    def placed(self):
        return len(self.corners) == 4


def picture_png(pgm_path, png_path):
    """The map picture as the app wants it: grey + alpha, 0 = wall, 255 = free, clear = never seen."""
    picture_from_array(np.asarray(Image.open(pgm_path).convert("L")), png_path)


def grid_to_pgm(grid):
    """A nav_msgs/OccupancyGrid (as slam_toolbox publishes /slam/map) -> the picture
    map_saver_cli would write: rows top first, walls 0, free 254, never seen 205."""
    w, h = grid.info.width, grid.info.height
    occ = np.asarray(grid.data, dtype=np.int16).reshape(h, w)[::-1]     # the grid's row 0 is the bottom
    v = np.full((h, w), 205, np.uint8)
    v[(occ >= 0) & (occ <= 25)] = 254
    v[occ >= 65] = 0
    return v


def save_grid(folder, name, grid):
    """Write <name>.pgm + <name>.yaml as map_saver_cli does (trinary), so the
    localization launch and MapStore read it like any other map."""
    v = grid_to_pgm(grid)
    Image.fromarray(v, "L").save(os.path.join(folder, name + ".pgm"))
    o = grid.info.origin.position
    tmp = os.path.join(folder, name + ".yaml.tmp")
    with open(tmp, "w") as f:
        f.write(f"image: {name}.pgm\nmode: trinary\nresolution: {grid.info.resolution}\n"
                f"origin: [{o.x}, {o.y}, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
    os.replace(tmp, os.path.join(folder, name + ".yaml"))


def picture_from_array(v, png_path):
    """map_saver values (walls 0, free 254, never seen 205) -> the app's PNG at png_path."""
    wall = v < WALL_BELOW
    # A wall is one 5 cm pixel: shrunk on a handset it vanishes. Drawn WALL_GROW
    # pixels thicker each way (the picture only; the map itself is untouched).
    for _ in range(WALL_GROW):
        w = wall.copy()
        w[1:, :] |= wall[:-1, :]
        w[:-1, :] |= wall[1:, :]
        w[:, 1:] |= wall[:, :-1]
        w[:, :-1] |= wall[:, 1:]
        wall = w
    grey = np.where(wall, 0, 255).astype(np.uint8)
    alpha = np.where(wall | (v > FREE_ABOVE), 255, 0).astype(np.uint8)
    tmp = png_path + ".tmp"
    Image.fromarray(np.dstack([grey, alpha]), "LA").save(tmp, "PNG", optimize=True)
    os.replace(tmp, png_path)


class MapStore:

    def __init__(self, folder):
        self.folder = os.path.expanduser(folder)
        os.makedirs(os.path.join(self.folder, PICTURES), exist_ok=True)
        self.maps = {}

    def path(self, name, suffix):
        return os.path.join(self.folder, name + suffix)

    def names(self):
        """Every map with a picture: a <name>.yaml that names an image beside it."""
        out = []
        for f in sorted(os.listdir(self.folder)):
            if f.endswith(".yaml") and not f.endswith("_geo.yaml"):
                name = f[:-len(".yaml")]
                if os.path.exists(self.path(name, ".pgm")):
                    out.append(name)
        return out

    def scan(self):
        """Read the folder again (an engineer may copy a map in). True when the list changed."""
        found = {}
        for name in self.names():
            try:
                found[name] = self.load(name)
            except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError):
                pass                                # a damaged map is left out, never fatal
        before = {n: (m.version, m.corners) for n, m in self.maps.items()}
        self.maps = found
        return before != {n: (m.version, m.corners) for n, m in found.items()}

    def load(self, name):
        with open(self.path(name, ".yaml")) as f:
            meta = yaml.safe_load(f)
        pgm = self.path(name, ".pgm")
        res = float(meta["resolution"])
        ox, oy = float(meta["origin"][0]), float(meta["origin"][1])
        stamp = int(os.path.getmtime(pgm))
        png = os.path.join(self.folder, PICTURES, f"{name}-{stamp}.png")
        if not os.path.exists(png):
            for old in os.listdir(os.path.join(self.folder, PICTURES)):
                if old.startswith(name + "-") and old[len(name) + 1:-4].isdigit():
                    os.remove(os.path.join(self.folder, PICTURES, old))
            picture_png(pgm, png)
        with Image.open(png) as img:
            w, h = img.size
        corners, err, tie = [], 0.0, ()
        geo = self.path(name, "_geo.yaml")
        if os.path.exists(geo):
            g = GeoMap(geo)
            tie = (g.lat0, g.lon0, g.rot, g.tx, g.ty)
            # The picture's corners in the map frame: origin is its bottom-left.
            top, right = oy + h * res, ox + w * res
            corners = [g.to_latlon(x, y) for x, y in ((ox, top), (right, top), (right, oy), (ox, oy))]
            with open(geo) as f:
                err = float(yaml.safe_load(f).get("residual_rms_m", 0.0))
        created = int(os.path.getmtime(self.path(name, ".yaml")) * 1000)
        return SavedMap(name, w, h, res, corners, err, created, png, tie)

    # -- naming a new map ----------------------------------------------------
    @staticmethod
    def safe_name(name):
        """A name that is also a file name: letters, digits, space, - and _ (others become -)."""
        clean = "".join(c if c.isalnum() or c in " -_" else "-" for c in name.strip())
        return clean.strip(" -")[:40]

    def name_taken(self, name):
        return name in self.maps or any(os.path.exists(self.path(name, s)) for s in SUFFIXES)

    def suggested_name(self):
        n = len(self.maps) + 1
        while self.name_taken(f"Map {n}"):
            n += 1
        return f"Map {n}"

    # -- what is shown under missions --------------------------------------
    def shown(self):
        try:
            with open(os.path.join(self.folder, SHOWN)) as f:
                name = f.read().strip()
        except OSError:
            return ""
        return name if name in self.maps else ""

    def show(self, name):
        """Draw `name` under missions ("" = none). False when there is no such map."""
        if name and name not in self.maps:
            return False
        tmp = os.path.join(self.folder, SHOWN + ".tmp")
        with open(tmp, "w") as f:
            f.write(name)
        os.replace(tmp, os.path.join(self.folder, SHOWN))
        return True

    def delete(self, name):
        """Remove a map's files for good. False when there is no such map."""
        if name not in self.maps:
            return False
        for suffix in SUFFIXES:
            try:
                os.remove(self.path(name, suffix))
            except FileNotFoundError:
                pass
        try:
            os.remove(self.maps[name].picture)
        except FileNotFoundError:
            pass
        if self.shown() == name:
            self.show("")
        del self.maps[name]
        return True
