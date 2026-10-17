#!/usr/bin/env python3
"""Read-only observer for RViz goals: no action client, planner calls or publishers.

Records the velocity chain, feedback/status/BT events, sensor ages and current
costmaps. Stop with SIGINT/SIGTERM; stopping never cancels a user's goal.
"""
import argparse
import csv
import json
import math
import signal
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.parameter import Parameter, parameter_value_to_python
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.srv import ListParameters, GetParameters, GetParameterTypes
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path as NavPath
from nav2_msgs.msg import CollisionMonitorState, BehaviorTreeLog, SpeedLimit
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatusArray
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from rosidl_runtime_py.convert import message_to_ordereddict
from tf2_ros import Buffer, TransformListener, TransformException


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=0, help='Wall seconds; 0 records until interrupted')
    args=parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('mppi_manual_observer', parameter_overrides=[Parameter('use_sim_time',value=True)])
    buffer=Buffer(); listener=TransformListener(buffer,node)
    latest={}; counts={}; stopped=False
    stream=(args.output/'samples.csv').open('w',newline=''); writer=csv.writer(stream)
    cols=['time','wall_time','x','y','tf_age','odom_vx','odom_vy','odom_age']
    for key in ['nav','smooth','collision','transformed','final']:
        cols += [key+'_vx',key+'_vy',key+'_wz',key+'_age']
    cols += ['scan_age','scan_nearest_m','collision_action','collision_state_receipt_age','watchdog_healthy','watchdog_receipt_age','distance_remaining','recoveries','goal_uuid','feedback_receipt_age']
    writer.writerow(cols)
    events=(args.output/'events.jsonl').open('w')
    def now(): return node.get_clock().now().nanoseconds/1e9
    def stamp(msg): return msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
    def event(kind,msg):
        events.write(json.dumps({'time':now(),'wall_time':time.time(),'topic':kind,'message':message_to_ordereddict(msg)})+'\n'); events.flush()
    def receive(key,msg):
        latest[key]=(msg,now()); counts[key]=counts.get(key,0)+1
    def subscription(cls,topic,key,qos=qos_profile_sensor_data):
        node.create_subscription(cls,topic,lambda m:receive(key,m),qos)
    for key,topic in [('nav','/cmd_vel_nav'),('smooth','/cmd_vel_smoothed'),('collision','/cmd_vel_collision'),('transformed','/cmd_vel_transformed'),('final','/cmd_vel')]:
        subscription(TwistStamped,topic,key)
    subscription(Odometry,'/odom_nav','odom'); subscription(LaserScan,'/scan','scan')
    subscription(Bool,'/cmd_vel_watchdog/healthy','watchdog')
    def collision(msg): receive('state',msg); event('/collision_monitor_state',msg)
    node.create_subscription(CollisionMonitorState,'/collision_monitor_state',collision,10)
    def feedback(msg): receive('feedback',msg)
    node.create_subscription(NavigateToPose.Impl.FeedbackMessage,'/navigate_to_pose/_action/feedback',feedback,qos_profile_sensor_data)
    latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
    seen={}
    def status(msg):
        for s in msg.status_list:
            uid=bytes(s.goal_info.goal_id.uuid).hex()
            if seen.get(uid)!=s.status:
                seen[uid]=s.status; event('/navigate_to_pose/_action/status',msg)
    node.create_subscription(GoalStatusArray,'/navigate_to_pose/_action/status',status,latched)
    for cls,topic in [(BehaviorTreeLog,'/behavior_tree_log'),(SpeedLimit,'/speed_limit'),(PoseStamped,'/goal_pose')]:
        node.create_subscription(cls,topic,lambda m,t=topic:event(t,m),10)
    def plan(msg):
        receive('plan',msg)
        event('/plan_endpoint',msg.poses[-1]) if msg.poses else None
        np.savez_compressed(args.output/'plan_latest.npz',xy=np.array([[p.pose.position.x,p.pose.position.y] for p in msg.poses]),time=stamp(msg),frame=msg.header.frame_id)
    node.create_subscription(NavPath,'/plan',plan,qos_profile_sensor_data)
    def costmap(msg,key):
        receive(key,msg)
        map_transform=[0.,0.,0.]
        if msg.header.frame_id!='map':
            try:
                tr=buffer.lookup_transform('map',msg.header.frame_id,rclpy.time.Time())
                q=tr.transform.rotation
                map_transform=[tr.transform.translation.x,tr.transform.translation.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
            except TransformException: map_transform=[float('nan')]*3
        np.savez_compressed(args.output/(key+'_latest.npz'),data=np.array(msg.data,dtype=np.int16).reshape(msg.info.height,msg.info.width),resolution=msg.info.resolution,origin=[msg.info.origin.position.x,msg.info.origin.position.y],time=stamp(msg),frame=msg.header.frame_id,map_transform=map_transform)
    for key,topic in [('global_costmap','/global_costmap/costmap'),('local_costmap','/local_costmap/costmap')]:
        node.create_subscription(OccupancyGrid,topic,lambda m,k=key:costmap(m,k),latched)
    def sample():
        t=now(); nan=float('nan'); x=y=tf_age=nan
        try:
            tr=buffer.lookup_transform('map','gimbal_yaw_fake',rclpy.time.Time())
            x=tr.transform.translation.x; y=tr.transform.translation.y; tf_age=t-stamp(tr)
        except TransformException: pass
        row=[t,time.time(),x,y,tf_age]
        odom=latest.get('odom',(None,0))[0]
        row += [odom.twist.twist.linear.x,odom.twist.twist.linear.y,t-stamp(odom)] if odom else [nan]*3
        for key in ['nav','smooth','collision','transformed','final']:
            msg=latest.get(key,(None,0))[0]
            row += [msg.twist.linear.x,msg.twist.linear.y,msg.twist.angular.z,t-stamp(msg)] if msg else [nan]*4
        scan=latest.get('scan',(None,0))[0]
        ranges=[v for v in scan.ranges if math.isfinite(v) and scan.range_min<=v<=scan.range_max] if scan else []
        row += [t-stamp(scan) if scan else nan,min(ranges) if ranges else nan]
        state,receipt=latest.get('state',(None,0)); row += [state.action_type if state else nan,t-receipt if state else nan]
        health,receipt=latest.get('watchdog',(None,0)); row += [int(health.data) if health else nan,t-receipt if health else nan]
        fb,receipt=latest.get('feedback',(None,0))
        row += [fb.feedback.distance_remaining,fb.feedback.number_of_recoveries,bytes(fb.goal_id.uuid).hex(),t-receipt] if fb else [nan,nan,'',nan]
        writer.writerow(row)
    def stop(signum,frame):
        nonlocal stopped
        stopped=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    def wait(future):
        until=time.monotonic()+5
        while not future.done() and time.monotonic()<until and not stopped: rclpy.spin_once(node,timeout_sec=.05)
        if not future.done():raise TimeoutError('parameter response timeout')
        return future.result()
    parameters={}
    for name in ['/controller_server','/velocity_smoother','/collision_monitor','/local_costmap/local_costmap','/global_costmap/global_costmap','/fake_vel_transform','/map_server','/amcl','/cmd_vel_watchdog']:
        clients=[]
        try:
            c=node.create_client(ListParameters,name+'/list_parameters');clients.append(c)
            if not c.wait_for_service(timeout_sec=2):raise TimeoutError('parameter service unavailable')
            names=wait(c.call_async(ListParameters.Request())).result.names
            c=node.create_client(GetParameterTypes,name+'/get_parameter_types');clients.append(c)
            req=GetParameterTypes.Request(names=names);types=wait(c.call_async(req)).types
            names=[k for k,v in zip(names,types) if v!=0]
            c=node.create_client(GetParameters,name+'/get_parameters');clients.append(c)
            values=wait(c.call_async(GetParameters.Request(names=names))).values
            parameters[name]=dict(zip(names,map(parameter_value_to_python,values)))
        except Exception as exc:parameters[name]={'error':str(exc)}
        finally:
            for c in clients:node.destroy_client(c)
    (args.output/'parameters.json').write_text(json.dumps(parameters,indent=2)+'\n')
    timer=node.create_timer(.05,sample); start=time.monotonic(); flushed=start
    print('READ-ONLY RECORDING '+str(args.output),flush=True)
    try:
        while rclpy.ok() and not stopped and (not args.duration or time.monotonic()-start<args.duration):
            rclpy.spin_once(node,timeout_sec=.05)
            if time.monotonic()-flushed>2:
                stream.flush();flushed=time.monotonic()
                (args.output/'health.json').write_text(json.dumps({'wall_time':time.time(),'ros_time':now(),'counts':counts,'status':seen},indent=2)+'\n')
    finally:
        stream.close();events.close();node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
