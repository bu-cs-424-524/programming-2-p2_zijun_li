#!/usr/bin/env python
import math
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseActionResult
from actionlib_msgs.msg import GoalStatus
from tf.transformations import quaternion_from_euler


class GoalNavigator(object):
    def __init__(self):
        rospy.init_node('p2a_nav_goals')

        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1, latch=True)
        self.pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_cb)
        self.result_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.result_cb)

        self.frame_id = rospy.get_param('~frame_id', 'map')
        self.goal_topic_delay = rospy.get_param('~goal_topic_delay', 1.0)
        self.goal_republish_interval = rospy.get_param('~goal_republish_interval', 2.0)
        self.position_tolerance = rospy.get_param('~position_tolerance', 0.35)
        self.yaw_tolerance = rospy.get_param('~yaw_tolerance', 0.35)
        self.wait_timeout = rospy.get_param('~wait_timeout', 180.0)
        self.goals = rospy.get_param('~goals', [])

        self.current_pose = None
        self.last_result_status = None

    def pose_cb(self, msg):
        self.current_pose = msg.pose.pose

    def result_cb(self, msg):
        self.last_result_status = msg.status.status

    def make_goal_msg(self, goal):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = float(goal['x'])
        msg.pose.position.y = float(goal['y'])
        msg.pose.position.z = 0.0

        yaw = float(goal.get('yaw', 0.0))
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        return msg

    @staticmethod
    def quat_to_yaw(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def pose_error_ok(self, goal):
        if self.current_pose is None:
            return False
        dx = self.current_pose.position.x - float(goal['x'])
        dy = self.current_pose.position.y - float(goal['y'])
        pos_err = math.sqrt(dx * dx + dy * dy)
        goal_yaw = float(goal.get('yaw', 0.0))
        cur_yaw = self.quat_to_yaw(self.current_pose.orientation)
        yaw_err = abs(math.atan2(math.sin(cur_yaw - goal_yaw), math.cos(cur_yaw - goal_yaw)))
        return pos_err <= self.position_tolerance and yaw_err <= self.yaw_tolerance

    def send_goal(self, goal):
        name = goal.get('name', 'unnamed_goal')
        msg = self.make_goal_msg(goal)
        rospy.sleep(self.goal_topic_delay)
        start_time = rospy.Time.now()
        next_publish = rospy.Time.now()
        self.last_result_status = None

        rospy.loginfo('Sending goal: %s (x=%.3f, y=%.3f, yaw=%.3f)', name, goal['x'], goal['y'], goal.get('yaw', 0.0))

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            if now >= next_publish:
                msg.header.stamp = now
                self.goal_pub.publish(msg)
                next_publish = now + rospy.Duration(self.goal_republish_interval)

            if self.pose_error_ok(goal):
                rospy.loginfo('Reached goal by tolerance check: %s', name)
                return True

            if self.last_result_status == GoalStatus.SUCCEEDED:
                rospy.loginfo('Move base reported success for goal: %s', name)
                return True

            if self.last_result_status in [GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.PREEMPTED]:
                rospy.logwarn('Move base failed on goal %s with status %s', name, str(self.last_result_status))
                return False

            if (now - start_time).to_sec() > self.wait_timeout:
                rospy.logwarn('Timed out waiting for goal: %s', name)
                return False

            rate.sleep()

    def run(self):
        if not self.goals:
            rospy.logerr('No goals supplied. Please edit config/p2a_params.yaml or override the goals parameter.')
            return

        rospy.loginfo('Waiting for /amcl_pose...')
        timeout_t = rospy.Time.now() + rospy.Duration(20.0)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.current_pose is None and rospy.Time.now() < timeout_t:
            rate.sleep()

        if self.current_pose is None:
            rospy.logwarn('No AMCL pose received yet. Navigation may still work once localization becomes available.')

        for idx, goal in enumerate(self.goals):
            ok = self.send_goal(goal)
            if not ok:
                rospy.logerr('Stopping goal sequence at index %d (%s).', idx, goal.get('name', 'unnamed_goal'))
                return
            rospy.sleep(1.0)

        rospy.loginfo('All navigation goals completed.')


if __name__ == '__main__':
    try:
        GoalNavigator().run()
    except rospy.ROSInterruptException:
        pass
