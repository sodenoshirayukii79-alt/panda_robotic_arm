#!/usr/bin/env python3

import cv2
import numpy as np
import rospy

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from panda_vision.msg import TargetPixel


class RedTargetDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.was_detected = False

        self.image_topic = rospy.get_param(
            "~image_topic", "/panda_camera/image_raw"
        )
        self.min_area = rospy.get_param("~min_area", 300.0)

        self.target_pub = rospy.Publisher(
            "/panda_vision/target_pixel",
            TargetPixel,
            queue_size=1
        )

        self.debug_pub = rospy.Publisher(
            "/panda_vision/debug_image",
            Image,
            queue_size=1
        )

        self.image_sub = rospy.Subscriber(
            self.image_topic,
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )

        rospy.loginfo("Red target detector started.")
        rospy.loginfo("Image topic: %s", self.image_topic)

    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="bgr8"
            )
        except CvBridgeError as exc:
            rospy.logerr_throttle(5.0, "CvBridge error: %s", str(exc))
            return

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # 红色跨越 HSV 色相区间的两端，需要两段阈值。
        lower_red_1 = np.array([0, 100, 70], dtype=np.uint8)
        upper_red_1 = np.array([10, 255, 255], dtype=np.uint8)

        lower_red_2 = np.array([170, 100, 70], dtype=np.uint8)
        upper_red_2 = np.array([180, 255, 255], dtype=np.uint8)

        mask_1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
        mask_2 = cv2.inRange(hsv, lower_red_2, upper_red_2)
        mask = cv2.bitwise_or(mask_1, mask_2)

        kernel = np.ones((5, 5), dtype=np.uint8)
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_OPEN, kernel
        )
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_CLOSE, kernel
        )

        result = TargetPixel()
        result.header = msg.header
        result.detected = False

        debug_image = image.copy()

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        if contours:
            contour = max(contours, key=cv2.contourArea)
            area = float(cv2.contourArea(contour))

            if area >= self.min_area:
                moments = cv2.moments(contour)

                if moments["m00"] > 0:
                    u = moments["m10"] / moments["m00"]
                    v = moments["m01"] / moments["m00"]

                    x, y, w, h = cv2.boundingRect(contour)

                    result.detected = True
                    result.center_u_px = float(u)
                    result.center_v_px = float(v)
                    result.area_px = area

                    cv2.rectangle(
                        debug_image,
                        (x, y),
                        (x + w, y + h),
                        (0, 255, 0),
                        2
                    )

                    cv2.circle(
                        debug_image,
                        (int(u), int(v)),
                        5,
                        (255, 0, 0),
                        -1
                    )

                    cv2.putText(
                        debug_image,
                        "Red target: u=%.1f v=%.1f" % (u, v),
                        (x, max(y - 10, 20)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        2
                    )

                    rospy.logdebug_throttle(
                        2.0,
                        "Target center: u=%.1f px, v=%.1f px, "
                        "area=%.0f px",
                        u, v, area
                    )

        if not result.detected:
            cv2.putText(
                debug_image,
                "No red target detected",
                (15, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2
            )

        # Log state changes
        if result.detected and not self.was_detected:
            rospy.loginfo("Red target first detected.")
        elif not result.detected and self.was_detected:
            rospy.loginfo("Red target lost.")

        self.was_detected = result.detected

        self.target_pub.publish(result)

        debug_msg = self.bridge.cv2_to_imgmsg(
            debug_image, encoding="bgr8"
        )
        debug_msg.header = msg.header
        self.debug_pub.publish(debug_msg)


if __name__ == "__main__":
    rospy.init_node("red_target_detector")
    RedTargetDetector()
    rospy.spin()