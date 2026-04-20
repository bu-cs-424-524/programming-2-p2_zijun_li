#!/usr/bin/env python
import math
import threading
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image


class BallFollower(object):
    def __init__(self):
        rospy.init_node('p2b_ball_follower')

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest_bgr = None
        self.latest_depth = None
        self.latest_bgr_stamp = None
        self.latest_depth_stamp = None

        self.rgb_topic = rospy.get_param('~rgb_topic', '/usb_cam/image_raw')
        self.depth_topic = rospy.get_param('~depth_topic', '/camera/depth/image_raw')
        self.cmd_topic = rospy.get_param('~cmd_topic', '/cmd_vel')
        self.debug_topic = rospy.get_param('~debug_topic', '/p2b/debug_image')

        self.target_distance = rospy.get_param('~target_distance', 1.0)
        self.distance_tolerance = rospy.get_param('~distance_tolerance', 0.12)
        self.angular_gain = rospy.get_param('~angular_gain', 1.6)
        self.linear_gain = rospy.get_param('~linear_gain', 0.7)
        self.max_linear = rospy.get_param('~max_linear', 0.22)
        self.max_angular = rospy.get_param('~max_angular', 1.0)
        self.min_blob_area = rospy.get_param('~min_blob_area', 250.0)
        self.control_rate = rospy.get_param('~control_rate', 10.0)
        self.search_angular_speed = rospy.get_param('~search_angular_speed', 0.35)
        self.center_deadband = rospy.get_param('~center_deadband', 0.06)
        self.depth_roi = int(rospy.get_param('~depth_roi', 5))

        self.lower_red1 = np.array(rospy.get_param('~lower_red1', [0, 120, 70]), dtype=np.uint8)
        self.upper_red1 = np.array(rospy.get_param('~upper_red1', [10, 255, 255]), dtype=np.uint8)
        self.lower_red2 = np.array(rospy.get_param('~lower_red2', [170, 120, 70]), dtype=np.uint8)
        self.upper_red2 = np.array(rospy.get_param('~upper_red2', [180, 255, 255]), dtype=np.uint8)

        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=1)
        self.debug_pub = rospy.Publisher(self.debug_topic, Image, queue_size=1)
        self.rgb_sub = rospy.Subscriber(self.rgb_topic, Image, self.rgb_cb, queue_size=1)
        self.depth_sub = rospy.Subscriber(self.depth_topic, Image, self.depth_cb, queue_size=1)

    def rgb_cb(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as exc:
            rospy.logerr('RGB conversion error: %s', str(exc))
            return
        with self.lock:
            self.latest_bgr = image
            self.latest_bgr_stamp = msg.header.stamp

    def depth_cb(self, msg):
        try:
            if msg.encoding == '32FC1':
                depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            else:
                depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as exc:
            rospy.logerr('Depth conversion error: %s', str(exc))
            return
        with self.lock:
            self.latest_depth = depth
            self.latest_depth_stamp = msg.header.stamp

    def get_frames(self):
        with self.lock:
            if self.latest_bgr is None or self.latest_depth is None:
                return None, None
            return self.latest_bgr.copy(), self.latest_depth.copy()

    def compute_depth_meters(self, depth_img, cx, cy):
        h, w = depth_img.shape[:2]
        half = self.depth_roi // 2
        x0 = max(0, cx - half)
        x1 = min(w, cx + half + 1)
        y0 = max(0, cy - half)
        y1 = min(h, cy + half + 1)
        patch = depth_img[y0:y1, x0:x1]

        patch = patch.astype(np.float32)
        valid = np.isfinite(patch) & (patch > 0.05)
        if not np.any(valid):
            return None

        vals = patch[valid]
        median_val = float(np.median(vals))
        if median_val > 20.0:
            median_val /= 1000.0
        return median_val

    def detect_red_ball(self, bgr):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        mask = cv2.GaussianBlur(mask, (9, 9), 0)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours_info = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = contours_info[0] if len(contours_info) == 2 else contours_info[1]
        if not contours:
            return None, mask

        contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(contour)
        if area < self.min_blob_area:
            return None, mask

        ((x, y), radius) = cv2.minEnclosingCircle(contour)
        moments = cv2.moments(contour)
        if moments['m00'] == 0:
            return None, mask
        cx = int(moments['m10'] / moments['m00'])
        cy = int(moments['m01'] / moments['m00'])

        return {
            'cx': cx,
            'cy': cy,
            'radius': radius,
            'area': area,
            'contour': contour,
        }, mask

    def make_twist(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        return msg

    def stop_robot(self):
        self.cmd_pub.publish(self.make_twist(0.0, 0.0))

    def run(self):
        rate = rospy.Rate(self.control_rate)
        while not rospy.is_shutdown():
            bgr, depth = self.get_frames()
            if bgr is None or depth is None:
                self.stop_robot()
                rate.sleep()
                continue

            detection, mask = self.detect_red_ball(bgr)
            debug = bgr.copy()
            h, w = debug.shape[:2]
            linear = 0.0
            angular = 0.0

            cv2.line(debug, (w // 2, 0), (w // 2, h), (255, 255, 0), 1)
            cv2.line(debug, (0, h // 2), (w, h // 2), (255, 255, 0), 1)

            if detection is None:
                angular = self.search_angular_speed
                cv2.putText(debug, 'Searching for red ball', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            else:
                cx = detection['cx']
                cy = detection['cy']
                depth_m = self.compute_depth_meters(depth, cx, cy)
                norm_err_x = float(cx - (w / 2.0)) / float(w / 2.0)

                cv2.drawContours(debug, [detection['contour']], -1, (0, 255, 0), 2)
                cv2.circle(debug, (cx, cy), int(max(detection['radius'], 5)), (255, 0, 0), 2)
                cv2.circle(debug, (cx, cy), 4, (0, 0, 255), -1)

                if abs(norm_err_x) > self.center_deadband:
                    angular = -self.angular_gain * norm_err_x
                angular = max(-self.max_angular, min(self.max_angular, angular))

                if depth_m is not None:
                    distance_err = depth_m - self.target_distance
                    if abs(distance_err) > self.distance_tolerance:
                        linear = self.linear_gain * distance_err
                    linear = max(-self.max_linear, min(self.max_linear, linear))
                    cv2.putText(debug, 'Depth: %.2f m' % depth_m, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                else:
                    linear = 0.0
                    cv2.putText(debug, 'Depth: unavailable', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

                cv2.putText(debug, 'Center error: %.3f' % norm_err_x, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(debug, 'Cmd linear=%.2f angular=%.2f' % (linear, angular), (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            self.cmd_pub.publish(self.make_twist(linear, angular))
            try:
                self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, encoding='bgr8'))
            except CvBridgeError as exc:
                rospy.logerr('Debug image publish error: %s', str(exc))

            rate.sleep()

        self.stop_robot()


if __name__ == '__main__':
    follower = BallFollower()
    try:
        follower.run()
    finally:
        follower.stop_robot()
