#!/bin/bash
# 杀死所有 XXU Gazebo 仿真相关进程

echo "正在清理仿真进程..."

pkill -9 -f "gz sim"
pkill -9 -f "parameter_bridge"
pkill -9 -f "robot_state_publisher"
pkill -9 -f "joint_state_publisher"
pkill -9 -f "small_point_lio"
pkill -9 -f "chassis_controller"
pkill -9 -f "imu_frame_republisher"
pkill -9 -f "scan_frame_republisher"
pkill -9 -f "pointcloud_frame_republisher"
pkill -9 -f "livox_pointcloud_shaper"
pkill -9 -f "pointcloud_to_laserscan"
pkill -9 -f "fake_vel_transform"
pkill -9 -f "cmd_vel_watchdog"
pkill -9 -f "cmd_vel_odometry"
pkill -9 -f "controller_manager"
pkill -9 -f "slam_toolbox"
pkill -9 -f "nav2_controller"
pkill -9 -f "nav2_smoother"
pkill -9 -f "nav2_planner"
pkill -9 -f "nav2_behaviors"
pkill -9 -f "nav2_bt_navigator"
pkill -9 -f "nav2_velocity_smoother"
pkill -9 -f "nav2_collision_monitor"
pkill -9 -f "nav2_lifecycle_manager"
pkill -9 -f "explore_lite"
pkill -9 -f "rviz2"
pkill -9 -f "ros2 launch"
pkill -9 -f "ros2 run"

sleep 1

remaining=$(ps aux | grep -E '(gz sim|rviz2|small_point|slam|nav2|explore|parameter_bridge|chassis|pointcloud|livox|fake_vel_transform|cmd_vel|controller_manage|lifecycle|spawner|robot_state|joint_state_pub|scan_frame|imu_frame|velocity_smoother|collision|bt_navigator|planner_server|behavior|smoother|ros2 launch|ros2 run)' | grep -v grep | grep -v gvfsd | wc -l)

if [ "$remaining" -eq 0 ]; then
    echo "所有仿真进程已清理完毕"
else
    echo "仍有 $remaining 个残留进程，尝试强制清理..."
    ps aux | grep -E '(gz sim|rviz2|small_point|slam|nav2|explore|parameter_bridge|chassis)' | grep -v grep | grep -v gvfsd | awk '{print $2}' | xargs kill -9 2>/dev/null
    echo "已完成"
fi
