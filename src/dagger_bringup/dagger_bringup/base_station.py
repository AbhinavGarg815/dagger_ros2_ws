#!/usr/bin/env python3
"""The RTK base station, for the app (through the gateway). Step R1.

    ros2 run dagger_bringup base_station

Talks MAVLink to the Blue Pill at the base (system 71) through the Cube: MAVROS
gives every message the Cube forwards on /uas1/mavlink_source and sends ours
from /uas1/mavlink_sink. The conversation is in DaggerGCS
docs/proposals/base-station.md; the MAVLink and the sites are in base_link.py
(tested offline: tools/test_base_link.py).

Out:  /base/status   dagger_msgs/BaseStatus, latched: heard, mode, site, survey progress
      /base/sites    dagger_msgs/BaseSiteList, latched: the saved sites
In:   /base/survey      std_srvs/Trigger         survey-in where the base stands
      /base/use_site    dagger_msgs/srv/MapName  fixed on a saved site's coordinates
      /base/save_site   dagger_msgs/srv/MapName  keep the survey just done ("" = suggested name)
      /base/delete_site dagger_msgs/srv/MapName  forget a site

Nothing here moves the rover. Changing the base moves where the rover thinks it
is, so the gateway refuses it while a job runs. Answers come at once; the
command is resent to the base until it is acknowledged.
"""
import time

import rclpy
from mavros_msgs.msg import Mavlink
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

from dagger_msgs.msg import BaseSite, BaseSiteList, BaseStatus
from dagger_msgs.srv import MapName

from .base_link import (BASE_SYSID, MODE_FIXED, MODE_SURVEYING, OUR_COMPID, OUR_SYSID, BaseLink, SiteStore,
                        from_payload64, to_payload64)


class BaseStationNode(Node):

    def __init__(self):
        super().__init__("base_station")
        self.declare_parameter("sites_file", "~/dagger_base_sites.json")
        self.declare_parameter("survey_min_s", 300.0)       # survey-in: at least this long ...
        self.declare_parameter("survey_accuracy_m", 2.0)    # ... and until this good
        p = self.get_parameter
        self.sites = SiteStore(p("sites_file").value)
        self.min_s, self.acc_m = float(p("survey_min_s").value), float(p("survey_accuracy_m").value)
        self.link = BaseLink()
        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        # Best effort in, reliable out: matches MAVROS whatever its own settings.
        hear = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=100)
        self.create_subscription(Mavlink, "/uas1/mavlink_source", self.on_mavlink, hear)
        self.sink = self.create_publisher(Mavlink, "/uas1/mavlink_sink", 10)
        self.status_pub = self.create_publisher(BaseStatus, "/base/status", latched)
        self.sites_pub = self.create_publisher(BaseSiteList, "/base/sites", latched)
        self.create_service(Trigger, "/base/survey", self.on_survey)
        self.create_service(MapName, "/base/use_site", self.on_use)
        self.create_service(MapName, "/base/save_site", self.on_save)
        self.create_service(MapName, "/base/delete_site", self.on_delete)
        self.last = None
        self.publish_sites()
        self.create_timer(0.2, self.tick)
        self.get_logger().info(f"{len(self.sites.sites)} base sites in {self.sites.path}")

    # ---------------- the base ----------------
    def on_mavlink(self, m):
        if m.sysid != BASE_SYSID:
            return
        if self.link.on_message(m.sysid, m.msgid, from_payload64(list(m.payload64), m.len), time.monotonic()):
            self.publish_status()

    def tick(self):
        now = time.monotonic()
        frame = self.link.due(now)
        if frame is not None:
            data, payload, crc = frame
            out = Mavlink(framing_status=Mavlink.FRAMING_OK, magic=Mavlink.MAVLINK_V20, len=len(payload),
                          incompat_flags=0, compat_flags=0, seq=data[4], sysid=OUR_SYSID, compid=OUR_COMPID,
                          msgid=75, checksum=crc, payload64=to_payload64(payload))
            out.header.stamp = self.get_clock().now().to_msg()
            self.sink.publish(out)
        self.publish_status(only_if_changed=True)

    def status(self):
        now = time.monotonic()
        lk, v = self.link, self.link.values
        st = BaseStatus(heard=lk.heard(now), mode=lk.mode, survey_target_m=self.acc_m,
                        suggested_site_name=self.sites.suggested_name())
        if lk.mode == MODE_SURVEYING:
            st.survey_accuracy_m = float(v.get("BS_ACC", 0.0))
            st.survey_s = int(v.get("BS_DUR", 0))
        if lk.mode == MODE_FIXED:
            site = self.sites.at(lk.position())
            st.site = site["name"] if site else ""
            st.survey_done = site is None and v.get("BS_DONE") == 1
        return st

    def publish_status(self, only_if_changed=False):
        st = self.status()
        key = (st.heard, st.mode, st.site, round(st.survey_accuracy_m, 2), st.survey_s, st.survey_done,
               st.suggested_site_name)
        if only_if_changed and key == self.last:
            return
        self.last = key
        self.status_pub.publish(st)

    def publish_sites(self):
        self.sites_pub.publish(BaseSiteList(sites=[
            BaseSite(name=s["name"], latitude=s["lat"], longitude=s["lon"], height_m=s["height_m"])
            for s in self.sites.sites]))

    # ---------------- from the gateway ----------------
    def on_survey(self, _req, res):
        self.link.survey(self.min_s, self.acc_m)
        self.get_logger().info(f"base: survey-in ({self.min_s:.0f} s, {self.acc_m:.1f} m)")
        res.success, res.message = True, ""
        return res

    def on_use(self, req, res):
        s = self.sites.get(req.name)
        if s is None:
            res.ok, res.error = False, "unknown site"
            return res
        self.link.fixed(s["lat"], s["lon"], s["height_m"])
        self.get_logger().info(f"base: fixed on '{s['name']}'")
        res.ok, res.error = True, ""
        return res

    def on_save(self, req, res):
        # A finished survey, or -- "save now" -- one still going: its mean so far, and the
        # base then stops surveying and uses it (fixed on the new site).
        pos = self.link.position() if self.link.values.get("BS_DONE") == 1 else None
        mean = self.link.survey_mean() if pos is None else None
        pos = pos or mean
        if pos is None or self.sites.at(pos) is not None:
            res.ok, res.error = False, "not surveyed"
            return res
        name = req.name.strip() or self.sites.suggested_name()
        if not self.sites.add(name, *pos):
            res.ok, res.error = False, "name taken"
            return res
        self.get_logger().info(f"base: site '{name}' saved at {pos[0]:.7f}, {pos[1]:.7f}, {pos[2]:.3f} m")
        if mean is not None:
            acc = self.link.values.get("BS_ACC", 0.0)
            self.link.fixed(*mean)
            self.get_logger().info(f"base: survey finished early ({acc:.2f} m), fixed on '{name}'")
        self.publish_sites()
        self.publish_status()
        res.ok, res.error = True, ""
        return res

    def on_delete(self, req, res):
        res.ok = self.sites.delete(req.name)
        res.error = "" if res.ok else "unknown site"
        if res.ok:
            self.get_logger().info(f"base: site '{req.name}' forgotten")
            self.publish_sites()
            self.publish_status()
        return res


def main():
    rclpy.init()
    node = BaseStationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
