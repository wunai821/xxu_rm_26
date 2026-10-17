#!/usr/bin/env python3
"""Read-only localization warnings; never publishes TF, poses or commands.

Measure a map->odom correction at the same current odom position so that the
translation of a rotated map transform is not mistaken for robot displacement.
"""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import signal
import time


def correction(previous, current, odom_xy):
    def mapped(tf):
        c, s = math.cos(tf['yaw']), math.sin(tf['yaw'])
        x, y = odom_xy
        return (tf['xy'][0]+c*x-s*y, tf['xy'][1]+s*x+c*y)
    dyaw = current['yaw']-previous['yaw']
    return {'position_m': math.dist(mapped(previous), mapped(current)),
            'yaw_deg': abs(math.degrees(math.atan2(math.sin(dyaw), math.cos(dyaw))))}


def odom_transition(previous, current, max_speed):
    """Flag missing samples and physically implausible observed displacement."""
    dt = current['stamp'] - previous['stamp']
    if dt <= 0:
        return None
    distance = math.dist(previous['xy'], current['xy'])
    speed = distance / dt
    reasons = []
    if dt > .5:
        reasons.append('odom_stamp_gap')
    if speed > max_speed:
        reasons.append('implausible_odom_displacement')
    return {'dt_s': dt, 'displacement_m': distance, 'average_speed_mps': speed,
            'reasons': reasons} if reasons else None


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--max-odom-speed',type=float,default=3.5,
                        help='Read-only displacement warning threshold, m/s')
    args=parser.parse_args()
    if not math.isfinite(args.max_odom_speed) or args.max_odom_speed <= 0:
        parser.error('--max-odom-speed must be finite and positive')
    args.output.mkdir(parents=True,exist_ok=False)
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from sensor_msgs.msg import LaserScan, Imu, JointState
    from tf2_msgs.msg import TFMessage
    from nav2_msgs.msg import ParticleCloud
    import numpy as np
    rclpy.init()
    node=rclpy.create_node('localization_integrity_monitor',parameter_overrides=[
        Parameter('use_sim_time',value=True)])
    events=(args.output/'events.jsonl').open('w')
    state={'odom':None,'map_tf':None,'amcl':None,'scan':None,'particles':None,
           'imu_raw':None,'imu_compensated':None,'joint':None}
    counts={k:0 for k in state}
    received={}
    recent_pose=deque(maxlen=100)
    imu_history={key:deque(maxlen=200) for key in ['imu_raw','imu_compensated']}
    initialized_at=-math.inf
    last_correction=None
    last_large_correction=None
    last_upstream_anomaly=None
    last_reasons=None
    max_correction=0.
    stopped=False
    def yaw(q):
        return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    def stamp(m):
        return m.header.stamp.sec+m.header.stamp.nanosec/1e9
    def now():
        return node.get_clock().now().nanoseconds/1e9
    def save_event(kind,details):
        events.write(json.dumps({'wall_time':time.time(),'ros_time':now(),
                                'kind':kind,**details})+'\n')
        events.flush()
    def received_msg(key,value):
        state[key]=value;counts[key]+=1;received[key]=time.monotonic()
    def odom(m):
        nonlocal last_upstream_anomaly
        p=m.pose.pose.position
        value={'stamp':stamp(m),'xy':[p.x,p.y],'yaw':yaw(m.pose.pose.orientation)}
        if state['odom']:
            transition=odom_transition(state['odom'],value,args.max_odom_speed)
            if transition:
                last_upstream_anomaly={'ros_time':now(),**transition}
                save_event('odom_input_anomaly',{'transition':transition,
                           'before':state['odom'],'after':value,
                           'imu_raw':state['imu_raw'],
                           'imu_compensated':state['imu_compensated']})
        received_msg('odom',value)
        recent_pose.append((time.monotonic(),value))
    def imu(m,key):
        nonlocal last_upstream_anomaly
        value={'stamp':stamp(m),
               'acceleration':[m.linear_acceleration.x,m.linear_acceleration.y,
                               m.linear_acceleration.z],
               'angular_velocity':[m.angular_velocity.x,m.angular_velocity.y,
                                   m.angular_velocity.z]}
        previous=state[key]
        if previous and value['stamp']-previous['stamp']>.1:
            gap=value['stamp']-previous['stamp']
            last_upstream_anomaly={'ros_time':now(),'topic':key,'gap_s':gap}
            save_event('imu_stamp_gap',{'topic':key,'gap_s':gap,'before':previous,
                       'after':value,'joint':state['joint'],
                       'recent_imu':list(imu_history[key])})
        imu_history[key].append(value)
        received_msg(key,value)
    def joint(m):
        if 'gimbal_joint' not in m.name:return
        index=m.name.index('gimbal_joint')
        received_msg('joint',{'stamp':stamp(m),
                     'position':m.position[index] if index<len(m.position) else None,
                     'velocity':m.velocity[index] if index<len(m.velocity) else None})
    def tf(m):
        nonlocal last_correction,last_large_correction,max_correction
        for tr in m.transforms:
            if tr.header.frame_id.lstrip('/')!='map' or tr.child_frame_id.lstrip('/')!='odom':
                continue
            p=tr.transform.translation
            value={'stamp':stamp(tr),'xy':[p.x,p.y],'yaw':yaw(tr.transform.rotation)}
            previous=state['map_tf']
            if previous and state['odom']:
                c=correction(previous,value,state['odom']['xy'])
                c.update(ros_time=now(),tf_stamp=value['stamp'])
                last_correction=c
                max_correction=max(max_correction,c['position_m'])
                if c['position_m']>.3 or c['yaw_deg']>10:
                    expected=time.monotonic()-initialized_at<2
                    if not expected and (c['position_m']>.5 or c['yaw_deg']>15):
                        last_large_correction=c
                    save_event('expected_initialization' if expected else 'map_correction',
                               {'correction':c,'severity':'info' if expected else
                                ('error' if c['position_m']>.5 or c['yaw_deg']>15 else 'warning'),
                                'odom':state['odom'],'scan':state['scan'],'particles':state['particles']})
            received_msg('map_tf',value)
    def amcl(m):
        p=m.pose.pose.position
        received_msg('amcl',{'stamp':stamp(m),'xy':[p.x,p.y],
                            'yaw':yaw(m.pose.pose.orientation),
                            'covariance_sigma_norm_m':math.sqrt(max(0.,m.pose.covariance[0]+m.pose.covariance[7]))})
    def initialpose(m):
        nonlocal initialized_at
        initialized_at=time.monotonic()
        save_event('initialpose_received',{'stamp':stamp(m)})
    def scan(m):
        a=np.asarray(m.ranges)
        step=max(1,(len(a)-1)//59)
        selected=a[::step]
        hits=int(np.sum(np.isfinite(selected)&(selected>=m.range_min)&(selected<m.range_max)))
        received_msg('scan',{'stamp':stamp(m),'selected_hits':hits})
    def particles(m):
        xy=np.array([[p.pose.position.x,p.pose.position.y] for p in m.particles])
        if len(xy):
            weights=np.array([p.weight for p in m.particles]);total=weights.sum()
            weights=weights/total if total>0 else np.ones(len(xy))/len(xy)
            mean=np.sum(xy*weights[:,None],axis=0)
            sigma=np.sqrt(np.sum((xy-mean)**2*weights[:,None],axis=0))
            received_msg('particles',{'stamp':stamp(m),'sigma_xy':sigma.tolist(),
                         'sigma_norm_m':float(np.linalg.norm(sigma)),'count':len(xy)})
    for cls,topic,callback in [(Odometry,'/odom',odom),(TFMessage,'/tf',tf),
            (PoseWithCovarianceStamped,'/amcl_pose',amcl),
            (PoseWithCovarianceStamped,'/initialpose',initialpose),
            (LaserScan,'/scan',scan),(ParticleCloud,'/particle_cloud',particles)]:
        qos=(QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,
                        reliability=ReliabilityPolicy.RELIABLE)
             if topic=='/amcl_pose' else qos_profile_sensor_data)
        node.create_subscription(cls,topic,callback,qos)
    for topic,key in [('/imu_raw','imu_raw'),
                      ('/imu_lio_compensated','imu_compensated')]:
        node.create_subscription(Imu,topic,lambda m,k=key:imu(m,k),qos_profile_sensor_data)
    node.create_subscription(JointState,'/joint_states',joint,qos_profile_sensor_data)
    def stop(*unused):
        nonlocal stopped
        stopped=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    started=written=time.monotonic()
    try:
        while rclpy.ok() and not stopped:
            rclpy.spin_once(node,timeout_sec=.05)
            t=time.monotonic()
            if t-written<1:continue
            written=t;reasons=[]
            if t-started>3:
                for key in ['odom','scan','map_tf','imu_raw','imu_compensated']:
                    timeout=.25 if key.startswith('imu_') else .75
                    if key not in received or t-received[key]>timeout:reasons.append(key+'_stale')
            if state['scan'] and state['scan']['selected_hits']<10:reasons.append('few_scan_hits')
            if state['particles'] and state['particles']['sigma_norm_m']>.75:reasons.append('particles_spread')
            elif state['amcl'] and state['amcl']['covariance_sigma_norm_m']>.75:reasons.append('pose_uncertainty')
            if last_large_correction and now()-last_large_correction['ros_time']<2:
                reasons.append('large_map_correction')
            if last_upstream_anomaly and now()-last_upstream_anomaly['ros_time']<2:
                reasons.append('upstream_input_anomaly')
            # Slow normal motion may legitimately wait for an AMCL update;
            # only warn after sustained movement and four seconds without one.
            older=next((v for tt,v in recent_pose if tt>=t-2),None)
            if older and state['odom']:
                moving=math.dist(older['xy'],state['odom']['xy'])>.25 or abs(
                    math.atan2(math.sin(older['yaw']-state['odom']['yaw']),
                               math.cos(older['yaw']-state['odom']['yaw'])))>.3
                if moving and (state['amcl'] is None or now()-state['amcl']['stamp']>4):
                    reasons.append('amcl_not_updating_while_moving')
            if reasons!=last_reasons:
                save_event('quality_state',{'reasons':reasons})
                last_reasons=reasons
            health={'wall_time':time.time(),'ros_time':now(),'read_only':True,
                    'status':'warning' if reasons else 'normal','reasons':reasons,
                    'counts':counts,'latest':state,'last_correction':last_correction,
                    'last_large_correction':last_large_correction,
                    'last_upstream_anomaly':last_upstream_anomaly,
                    'max_correction_m_since_start':max_correction}
            tmp=args.output/'health.tmp';tmp.write_text(json.dumps(health,indent=2)+'\n')
            tmp.replace(args.output/'health.json')
    finally:
        events.close();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
