#!/usr/bin/env python3
"""Stop distance from a pf_* bag: time + distance from the first FCU setpoint drop (>0.1 -> 0) to rest.
    python3 tools/stop_distance.py ~/dagger_bags/pf_<name> [...]
"""
import sys, math, time, bisect, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
def load(p):
    r=rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=p,storage_id="mcap"),rosbag2_py.ConverterOptions("cdr","cdr"))
    ty={t.name:t.type for t in r.get_all_topics_and_types()}
    W=["/mavros/setpoint_velocity/cmd_vel_unstamped","/odometry/filtered","/mavros/rc/out","/mavros/state","/cmd_vel_nav_raw","/mavros/local_position/velocity_body"]
    D={k:[] for k in W}
    while r.has_next():
        t,d,ts=r.read_next()
        if t in D: D[t].append((ts*1e-9,deserialize_message(d,get_message(ty[t]))))
    return D
def at(D,k,t):
    a=D[k]; i=bisect.bisect_right([x[0] for x in a],t)-1
    return a[i][1] if i>=0 else None
for p in sys.argv[1:]:
    D=load(p); sp=D["/mavros/setpoint_velocity/cmd_vel_unstamped"]
    stops=[sp[i][0] for i in range(1,len(sp)) if sp[i-1][1].linear.x>0.1 and sp[i][1].linear.x<0.01]
    print(f"=== {p.split('/')[-1]}  zero-command events: {len(stops)}")
    t_stop=stops[0]
    # cruise before stop
    odv=[(t,m.twist.twist.linear.x) for t,m in D["/odometry/filtered"] if t_stop-3<t<t_stop]
    rcs=[(m.channels[0],m.channels[2]) for t,m in D["/mavros/rc/out"] if t_stop-3<t<t_stop]
    print(f"  last 3 s before zero: v mean {sum(v for _,v in odv)/len(odv):.2f} m/s, PWM L/R {sum(a for a,_ in rcs)/len(rcs):.0f}/{sum(b for _,b in rcs)/len(rcs):.0f}")
    o0=at(D,"/odometry/filtered",t_stop); x0,y0=o0.pose.pose.position.x,o0.pose.pose.position.y
    t_rest=None
    for t,m in D["/odometry/filtered"]:
        if t>t_stop and abs(m.twist.twist.linear.x)<0.02:
            t_rest=t; break
    o1=at(D,"/odometry/filtered",t_rest); 
    dist=math.hypot(o1.pose.pose.position.x-x0,o1.pose.pose.position.y-y0)
    st=at(D,"/mavros/state",t_rest)
    print(f"  zero command -> rest: {t_rest-t_stop:.2f} s, rolled {dist:.2f} m (armed at rest: {st.armed if st else '?'})")
    t=t_stop-0.5
    while t<t_rest+1.0:
        o=at(D,"/odometry/filtered",t); rc=at(D,"/mavros/rc/out",t); s=at(D,"/mavros/setpoint_velocity/cmd_vel_unstamped",t); stt=at(D,"/mavros/state",t)
        print(f"   {t-t_stop:+5.2f}s sp={s.linear.x:.2f} v={o.twist.twist.linear.x:.2f} d={math.hypot(o.pose.pose.position.x-x0,o.pose.pose.position.y-y0):.2f} PWM {rc.channels[0]}/{rc.channels[2]} {'A' if stt and stt.armed else 'd'}")
        t+=0.25
