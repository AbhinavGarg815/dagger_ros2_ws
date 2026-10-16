#!/usr/bin/env python3
"""Saved maps: the map manager's /maps/list -> the contract's MapList. NO ROS.

    python3 src/dagger_gateway/test/test_maps.py     (or pytest)
"""
import os
import sys
from types import SimpleNamespace as NS

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
from dagger_gateway.maps import map_list_pb, mapping_pb  # noqa: E402


def info(name, placed=True, version=1792047715):
    lat = [26.7481368, 26.7481499, 26.7477308, 26.7477177] if placed else []
    lon = [75.8550696, 75.8555780, 75.8555914, 75.8550830] if placed else []
    return NS(name=name, width_m=50.6, height_m=46.45, placed=placed, corner_lat=lat, corner_lon=lon,
              placed_error_m=1.09 if placed else 0.0, picture=f"/x/{name}.png", picture_width_px=1012,
              picture_height_px=929, picture_version=version, created_ms=1792047715000)


def test_no_list_yet_is_an_empty_list():
    out = map_list_pb(None)
    assert list(out.maps) == [] and out.shown == ""


def test_a_placed_map_has_its_corners_and_a_picture_path():
    out = map_list_pb(NS(maps=[info("lane3fix"), info("Work shop", placed=False)], shown="lane3fix"))
    a, b = out.maps
    assert a.placed and len(a.corners) == 4
    assert abs(a.corners[0].latitude_deg - 26.7481368) < 1e-9 and abs(a.corners[2].longitude_deg - 75.8555914) < 1e-9
    assert a.picture_path == "/maps/lane3fix.png?v=1792047715"
    assert not b.placed and list(b.corners) == []
    assert b.picture_path == "/maps/Work%20shop.png?v=1792047715", "a name with a space is quoted"
    assert out.shown == "lane3fix"


def test_a_shown_map_no_longer_listed_is_not_shown():
    out = map_list_pb(NS(maps=[info("lane3fix")], shown="gone"))
    assert out.shown == ""



def test_the_map_being_made():
    assert mapping_pb(None) is None and mapping_pb(NS(active=False)) is None
    live = info("", placed=False, version=17)
    st = mapping_pb(NS(active=True, saving=False, suggested_name="Map 3", live=live))
    assert st.suggested_name == "Map 3" and not st.saving
    assert st.map.picture_path == "/maps/~live.png?v=17" and not st.map.placed
    placed = mapping_pb(NS(active=True, saving=True, suggested_name="Map 3", live=info("", version=18)))
    assert placed.saving and placed.map.placed and len(placed.map.corners) == 4

if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok   ", name)
    print("all passed")
