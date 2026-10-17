#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
path_logger.py —— 把 /current_state 与 /desired_state 累积成两条 nav_msgs/Path，
供 rviz 画「期望圆 vs 实际轨迹」。

为什么需要它
------------
官方 `traj_publisher.cpp` 只把期望位姿发成 **TF（world→av-desired）** 和
`/desired_state`（MultiDOFJointTrajectoryPoint，**没有 header**），而
`/current_state` 是 nav_msgs/Odometry —— rviz 的 Odometry 显示只画当前位姿，
不留痕。所以"飞满一圈"在官方 rviz 配置里根本看不出来。
本节点是**纯观察者**：只订阅、只发布新的 Path 话题，不改动任何交付代码。

发布：
    /desired_path   nav_msgs/Path  （期望圆周，蓝）
    /drone_path     nav_msgs/Path  （实际轨迹，红）

用法：
    rosrun controller_pkg path_logger.py       # 或直接 python3 path_logger.py
"""

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint

MAX_POSES = 20000          # 12.57 s 一圈，20 Hz 采样 → 一圈 250 点，够存 80 圈
PUBLISH_EVERY = 0.05       # 20 Hz 重发（rviz Path 只认整条消息）


class PathLogger(object):
    def __init__(self):
        self.drone = Path()
        self.drone.header.frame_id = "world"
        self.desired = Path()
        self.desired.header.frame_id = "world"

        self.pub_drone = rospy.Publisher("/drone_path", Path, queue_size=1, latch=True)
        self.pub_desired = rospy.Publisher("/desired_path", Path, queue_size=1, latch=True)

        rospy.Subscriber("/current_state", Odometry, self.on_current, queue_size=50)
        rospy.Subscriber("/desired_state", MultiDOFJointTrajectoryPoint,
                         self.on_desired, queue_size=50)
        rospy.Timer(rospy.Duration(PUBLISH_EVERY), self.publish)
        rospy.loginfo("[path_logger] 已启动：/current_state → /drone_path, "
                      "/desired_state → /desired_path")

    # ------------------------------------------------------------------
    def on_current(self, msg):
        ps = PoseStamped()
        ps.header = msg.header
        if not ps.header.frame_id:
            ps.header.frame_id = "world"
        ps.header.stamp = rospy.Time.now()
        ps.pose = msg.pose.pose
        self._append(self.drone, ps)

    def on_desired(self, msg):
        if not msg.transforms:
            return
        ps = PoseStamped()
        ps.header.frame_id = "world"
        ps.header.stamp = rospy.Time.now()
        tr = msg.transforms[0]
        ps.pose.position = tr.translation
        ps.pose.orientation = tr.rotation
        self._append(self.desired, ps)

    @staticmethod
    def _append(path, ps):
        # 按位移去抖：静止时不堆重复点（否则 Path 会越来越重、rviz 重绘变慢）
        if path.poses:
            p0 = path.poses[-1].pose.position
            p1 = ps.pose.position
            if (abs(p1.x - p0.x) + abs(p1.y - p0.y) + abs(p1.z - p0.z)) < 0.02:
                return
        path.poses.append(ps)
        if len(path.poses) > MAX_POSES:
            del path.poses[: len(path.poses) - MAX_POSES]

    def publish(self, _evt=None):
        now = rospy.Time.now()
        self.drone.header.stamp = now
        self.desired.header.stamp = now
        self.pub_drone.publish(self.drone)
        self.pub_desired.publish(self.desired)


if __name__ == "__main__":
    rospy.init_node("path_logger", anonymous=False)
    PathLogger()
    rospy.spin()
