#!/usr/bin/env python3
import math

import rospy
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped
from panda_vision.msg import TargetPixel


class PixelToWorld:
    def __init__(self):
        # 当前仿真中固定相机的世界坐标
        self.camera_x = float(rospy.get_param("~camera_x", 0.45))
        self.camera_y = float(rospy.get_param("~camera_y", 0.0))
        self.camera_z = float(rospy.get_param("~camera_z", 1.20))

        # 红色方块顶部所在的水平平面
        self.target_plane_z = float(
            rospy.get_param("~target_plane_z", 0.08)
        )

        self.camera_info = None
        self.was_detected = False

        self.pose_pub = rospy.Publisher(
            "/panda_vision/target_pose",
            PoseStamped,
            queue_size=1
        )

        self.info_sub = rospy.Subscriber(
            "/panda_camera/camera_info",
            CameraInfo,
            self.camera_info_callback,
            queue_size=1
        )

        self.target_sub = rospy.Subscriber(
            "/panda_vision/target_pixel",
            TargetPixel,
            self.target_callback,
            queue_size=1
        )

        rospy.loginfo("Pixel-to-world node started.")

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def target_callback(self, msg):
        if not msg.detected:
            if self.was_detected:
                rospy.loginfo("Target world coordinates lost.")
            self.was_detected = False
            return

        if self.camera_info is None:
            rospy.logwarn_throttle(
                5.0, "Waiting for camera calibration information."
            )
            return

        k = self.camera_info.K
        fx, cx = k[0], k[2]
        fy, cy = k[4], k[5]

        if not all(math.isfinite(v) for v in [fx, fy, cx, cy]):
            rospy.logwarn("Camera intrinsics contain invalid values.")
            return

        if fx <= 0.0 or fy <= 0.0:
            rospy.logwarn("Invalid camera focal length.")
            return

        depth = self.camera_z - self.target_plane_z
        if depth <= 0.0:
            rospy.logerr("Target plane must be below the camera.")
            return

        # 针对当前竖直向下的相机安装姿态进行平面反投影。
        # 像素 u 向右，对应世界 y 减小；
        # 像素 v 向下，对应世界 x 减小。
        x_world = self.camera_x - (
            (msg.center_v_px - cy) * depth / fy
        )
        y_world = self.camera_y - (
            (msg.center_u_px - cx) * depth / fx
        )
        z_world = self.target_plane_z

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "world"

        pose.pose.position.x = x_world
        pose.pose.position.y = y_world
        pose.pose.position.z = z_world

        # 这里表示目标点，不代表已经计算好夹爪抓取姿态。
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        self.pose_pub.publish(pose)

        # Log state change on first detection
        if not self.was_detected:
            rospy.loginfo(
                "First target world position: x=%.3f, y=%.3f, z=%.3f",
                x_world, y_world, z_world
            )
            self.was_detected = True
        else:
            # Debug logging for continuous updates
            rospy.logdebug_throttle(
                2.0,
                "Target world position: x=%.3f, y=%.3f, z=%.3f",
                x_world, y_world, z_world
            )


if __name__ == "__main__":
    rospy.init_node("pixel_to_world")
    PixelToWorld()
    rospy.spin()
