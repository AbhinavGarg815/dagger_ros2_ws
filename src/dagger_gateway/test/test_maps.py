#!/usr/bin/env python3
"""Saved maps: the map manager's /maps/list -> the contract's MapList. NO ROS.

    python3 src/dagger_gateway/test/test_maps.py     (or pytest)
"""
import math
import os
import sys
from types import SimpleNamespace as NS

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
from dagger_gateway.maps import SlamHeard, map_list_pb, mapping_pb, placed_map, to_latlon, to_map  # noqa: E402


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


def test_lat_lon_to_a_map_and_back():
    geo = (26.7479, 75.8553, math.radians(30.0), 4.0, -2.0)
    for x, y in ((0.0, 0.0), (12.5, -7.0), (-3.0, 20.0)):
        lat, lon = to_latlon(geo, x, y)
        x2, y2 = to_map(geo, lat, lon)
        assert abs(x2 - x) < 1e-6 and abs(y2 - y) < 1e-6


def test_the_same_tie_as_the_map_manager():
    # dagger_bringup.geo_map.GeoMap reads the same five numbers from <name>_geo.yaml.
    bringup = os.path.join(HERE, "..", "..", "dagger_bringup")
    if not os.path.isdir(bringup):
        return
    sys.path.insert(0, bringup)
    import tempfile
    from dagger_bringup.geo_map import GeoMap
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "x_geo.yaml")
        open(p, "w").write("datum_lat: 26.7479\ndatum_lon: 75.8553\nyaw_deg: 30.0\ntx: 4.0\nty: -2.0\n")
        g = GeoMap(p)
        geo = (g.lat0, g.lon0, g.rot, g.tx, g.ty)
        for lat, lon in ((26.74795, 75.85535), (26.7481, 75.8550)):
            a, b = g.to_map(lat, lon)[:2], to_map(geo, lat, lon)
            assert abs(a[0] - b[0]) < 1e-9 and abs(a[1] - b[1]) < 1e-9


def test_points_on_a_picture_and_on_its_map():
    # Step D2: the picture's bottom-left is the map's .yaml origin; headings clockwise from up.
    from dagger_gateway.maps import from_picture, map_info, picture_heading, to_picture
    m = NS(name="Workshop", origin_x=-12.5, origin_y=-4.0)
    assert from_picture(m, 4.25, 7.5) == (-8.25, 3.5)
    assert to_picture(m, *from_picture(m, 4.25, 7.5)) == (4.25, 7.5)
    assert picture_heading(0.0) == 90.0, "the map's +x is the picture's right"
    assert picture_heading(math.pi / 2) == 0.0, "+y is up"
    assert abs(picture_heading(math.pi) - 270.0) < 1e-9
    assert map_info(NS(maps=[m]), "Workshop") is m and map_info(NS(maps=[m]), "Yard") is None
    assert map_info(None, "Workshop") is None


def test_only_a_placed_map_with_its_tie_takes_missions():
    yard = info("Yard")
    yard.geo = [26.7479, 75.8553, 0.5, 1.0, 2.0]
    work = info("Workshop", placed=False)
    work.geo = []
    ml = NS(maps=[yard, work], shown="", localizing="")
    assert placed_map(ml, "Yard") is yard
    assert placed_map(ml, "Workshop") is None and placed_map(ml, "Nowhere") is None and placed_map(None, "Yard") is None

def test_found_on_the_map_survives_a_gateway_restart_in_either_order():
    # Owner, 2026-10-23: after a gateway restart with Map 5 found, Place on the map stayed red.
    for first in ("verdict", "list"):
        s = SlamHeard()
        if first == "verdict":
            s.localized_heard, s.finding_heard = True, 5
            assert not s.localized, "no map known yet: not found"
            assert s.map_in_use("Map 5") is False, "the first list is not a change"
        else:
            assert s.map_in_use("Map 5") is False
            s.localized_heard, s.finding_heard = True, 5
        assert s.localized and s.finding == 5, f"found ({first} first)"
    # a real change of map drops the old verdicts; the new localization says for itself
    assert s.map_in_use("Map 4") is True
    assert not s.localized and s.finding is None
    s.localized_heard = True
    assert s.localized
    # no map in use: never found, whatever was heard
    assert s.map_in_use("") is True and not s.localized
    s.localized_heard, s.finding_heard = True, 2
    assert not s.localized and s.finding is None


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok   ", name)
    print("all passed")
