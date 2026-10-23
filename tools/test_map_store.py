#!/usr/bin/env python3
"""Offline test of dagger_bringup.map_store (the operator's saved maps). NO ROS, NO ROVER.

    python3 tools/test_map_store.py        (or: python3 -m pytest tools/test_map_store.py)

Builds maps in a temporary folder: the list, the PNG (walls, free, never seen),
the four corners from <name>_geo.yaml, which map is shown, delete (only that
map's files: "lane3" leaves "lane3fix" alone), and a damaged map left out.
"""
import math
import os
import sys
import tempfile

import numpy as np
from PIL import Image

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src", "dagger_bringup"))
from dagger_bringup.geo_map import enu_scale  # noqa: E402
from dagger_bringup.map_store import MapStore  # noqa: E402

LAT0, LON0 = 26.747944406, 75.855284239


def make_map(folder, name, w=40, h=20, res=0.5, origin=(-10.0, -5.0), geo=None):
    """A w x h map: free inside, a wall along the top row, a never-seen corner."""
    v = np.full((h, w), 254, np.uint8)
    v[0, :] = 0                       # wall along the top
    v[h - 4:, :4] = 205               # never seen, bottom-left
    Image.fromarray(v, "L").save(os.path.join(folder, name + ".pgm"))
    with open(os.path.join(folder, name + ".yaml"), "w") as f:
        f.write(f"image: {name}.pgm\nmode: trinary\nresolution: {res}\n"
                f"origin: [{origin[0]}, {origin[1]}, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
    for ext in (".posegraph", ".data", "_view.png"):
        open(os.path.join(folder, name + ext), "w").write("x")
    if geo is not None:
        yaw, tx, ty = geo
        with open(os.path.join(folder, name + "_geo.yaml"), "w") as f:
            f.write(f"datum_lat: {LAT0}\ndatum_lon: {LON0}\nyaw_deg: {yaw}\ntx: {tx}\nty: {ty}\n"
                    f"residual_rms_m: 1.09\nresidual_max_m: 1.76\n")


def test_lists_maps_with_size_and_a_picture():
    with tempfile.TemporaryDirectory() as d:
        make_map(d, "Yard", geo=(0.0, 0.0, 0.0))
        make_map(d, "Workshop")
        store = MapStore(d)
        assert store.scan()
        assert sorted(store.maps) == ["Workshop", "Yard"]
        y = store.maps["Yard"]
        assert (y.width_px, y.height_px) == (40, 20) and (y.width_m, y.height_m) == (20.0, 10.0)
        assert y.origin == (-10.0, -5.0), "the picture's bottom-left on the map (Step D2: I am here)"
        img = np.asarray(Image.open(y.picture))
        assert img.shape == (20, 40, 2)
        assert tuple(img[0, 10]) == (0, 255), "a wall: black, solid"
        assert tuple(img[2, 10]) == (0, 255), "drawn thicker: 2 pixels below the wall too"
        assert tuple(img[10, 10]) == (255, 255), "free: white, solid"
        assert img[19, 0][1] == 0, "never seen: clear"
        assert not store.scan(), "nothing changed"


def test_corners_come_from_the_geo_file():
    with tempfile.TemporaryDirectory() as d:
        make_map(d, "Yard", geo=(0.0, 0.0, 0.0))
        make_map(d, "Turned", geo=(90.0, 3.0, 4.0))
        make_map(d, "Workshop")
        store = MapStore(d)
        store.scan()
        kx, ky = enu_scale(LAT0)
        tl, tr, br, bl = store.maps["Yard"].corners
        # origin (-10, -5) is the bottom-left; 20 x 10 m; no turn, no shift: map = ENU.
        assert math.isclose(bl[0], LAT0 - 5 / ky, abs_tol=1e-9) and math.isclose(bl[1], LON0 - 10 / kx, abs_tol=1e-9)
        assert math.isclose(tl[0], LAT0 + 5 / ky, abs_tol=1e-9)
        assert math.isclose(tr[1], LON0 + 10 / kx, abs_tol=1e-9) and math.isclose(br[0], bl[0], abs_tol=1e-12)
        # Turned 90 degrees: the map's x axis points north; then shifted 3 m east, 4 m north.
        tl, tr, br, bl = store.maps["Turned"].corners
        assert math.isclose(bl[0], LAT0 + (-10 + 4) / ky, abs_tol=1e-9)
        assert math.isclose(bl[1], LON0 + (5 + 3) / kx, abs_tol=1e-9)
        assert store.maps["Yard"].placed and store.maps["Yard"].placed_error_m == 1.09
        assert not store.maps["Workshop"].placed and store.maps["Workshop"].corners == []


def test_shown_is_kept_and_only_a_known_map():
    with tempfile.TemporaryDirectory() as d:
        make_map(d, "Yard")
        store = MapStore(d)
        store.scan()
        assert store.shown() == ""
        assert not store.show("Nowhere")
        assert store.show("Yard") and store.shown() == "Yard"
        again = MapStore(d)                         # a restart
        again.scan()
        assert again.shown() == "Yard"
        assert again.show("") and again.shown() == ""


def test_delete_removes_only_that_maps_files():
    with tempfile.TemporaryDirectory() as d:
        make_map(d, "lane3", geo=(0.0, 0.0, 0.0))
        make_map(d, "lane3fix")
        store = MapStore(d)
        store.scan()
        store.show("lane3")
        assert store.delete("lane3") and not store.delete("lane3")
        left = sorted(f for f in os.listdir(d) if not f.startswith("."))
        assert left == sorted("lane3fix" + s for s in (".yaml", ".pgm", ".posegraph", ".data", "_view.png"))
        assert store.shown() == "", "the shown map deleted: nothing shown"
        assert list(store.maps) == ["lane3fix"]
        pictures = os.listdir(os.path.join(d, ".pictures"))
        assert len(pictures) == 1 and pictures[0].startswith("lane3fix-")


def test_a_new_map_is_found_and_a_damaged_one_left_out():
    with tempfile.TemporaryDirectory() as d:
        store = MapStore(d)
        assert not store.scan() and store.maps == {}
        make_map(d, "Yard")
        open(os.path.join(d, "Bad.yaml"), "w").write("resolution: [not a number\n")
        open(os.path.join(d, "Bad.pgm"), "w").write("not a picture")
        assert store.scan() and list(store.maps) == ["Yard"]


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok   ", name)
    print("all passed")
