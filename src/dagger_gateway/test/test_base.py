#!/usr/bin/env python3
"""The RTK base station: base_station's topics -> the contract's BaseStation and BaseSites. NO ROS.

    python3 src/dagger_gateway/test/test_base.py     (or pytest)
"""
import os
import sys
from types import SimpleNamespace as NS

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
from dagger_gateway import api_pb2 as pb  # noqa: E402
from dagger_gateway.base import base_pb, rtk_of, sites_pb  # noqa: E402


def test_no_status_yet_is_not_heard():
    assert not base_pb(None).heard and list(sites_pb(None).sites) == []


def test_status_and_sites():
    st = NS(heard=True, mode=2, site="", survey_accuracy_m=2.4, survey_target_m=2.0, survey_s=95,
            survey_done=False, suggested_site_name="Site 2")
    b = base_pb(st)
    assert b.heard and b.mode == pb.BASE_MODE_SURVEYING and abs(b.survey_accuracy_m - 2.4) < 1e-6
    assert b.survey_s == 95 and b.suggested_site_name == "Site 2"
    fixed = base_pb(NS(**dict(vars(st), mode=3, site="Yard base")))
    assert fixed.mode == pb.BASE_MODE_FIXED and fixed.site == "Yard base"
    out = sites_pb(NS(sites=[NS(name="Yard base", latitude=26.7478123, longitude=75.8554012, height_m=418.237)]))
    assert out.sites[0].name == "Yard base" and out.sites[0].height_m == 418.237


def test_rtk_from_the_cubes_fix_type():
    assert rtk_of(6) == pb.RTK_STATUS_FIXED and rtk_of(5) == pb.RTK_STATUS_FLOAT
    assert rtk_of(3) == pb.RTK_STATUS_NONE and rtk_of(0) == pb.RTK_STATUS_NONE


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok   ", name)
    print("all passed")
