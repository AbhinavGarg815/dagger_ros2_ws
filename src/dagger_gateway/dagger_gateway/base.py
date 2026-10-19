"""The RTK base station for the app (Step R1): the base_station node's topics -> the contract.

dagger_bringup/base_station owns the base (MAVLink to the Blue Pill through the
Cube) and its sites; the gateway only translates and decides when a change is
allowed (not while a job runs: the rover's position would jump).
"""
from . import api_pb2 as pb

MODES = {1: pb.BASE_MODE_NO_FIX, 2: pb.BASE_MODE_SURVEYING, 3: pb.BASE_MODE_FIXED}


def base_pb(st):
    """dagger_msgs/BaseStatus -> the contract's BaseStation (not heard when none yet)."""
    if st is None:
        return pb.BaseStation(heard=False)
    return pb.BaseStation(heard=st.heard, mode=MODES.get(st.mode, pb.BASE_MODE_UNSPECIFIED), site=st.site,
                          survey_accuracy_m=st.survey_accuracy_m, survey_target_m=st.survey_target_m,
                          survey_s=st.survey_s, survey_done=st.survey_done,
                          suggested_site_name=st.suggested_site_name)


def sites_pb(msg):
    """dagger_msgs/BaseSiteList -> the contract's BaseSites."""
    out = pb.BaseSites()
    for s in (msg.sites if msg is not None else []):
        out.sites.append(pb.BaseSite(name=s.name, latitude_deg=s.latitude, longitude_deg=s.longitude,
                                     height_m=s.height_m))
    return out


def rtk_of(fix_type):
    """GPS_RAW_INT fix type -> RtkStatus: 6 fixed, 5 float, else none."""
    return {6: pb.RTK_STATUS_FIXED, 5: pb.RTK_STATUS_FLOAT}.get(fix_type, pb.RTK_STATUS_NONE)
