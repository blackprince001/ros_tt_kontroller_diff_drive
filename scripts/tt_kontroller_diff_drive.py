#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
import threading

import tf.transformations as Transform

class TTKontrollerDiffDrive:
    def __init__(self):
        rospy.init_node('tt_kontroller_diff_drive', anonymous=True)

        self.command_topic = rospy.get_param("~commandTopic", "cmd_vel")
        self.odometry_topic = rospy.get_param("~odometryTopic", "odom")

        self.odometry_frame = rospy.get_param("~odometryFrame", "odom")
        self.robot_base_frame = rospy.get_param("~robotBaseFrame", "base_link")

        self.wheel_separation = rospy.get_param("~wheelSeparation", 0.34)
        self.wheel_diameter = rospy.get_param("~wheelDiameter", 0.15)
        self.wheel_accel = rospy.get_param("~wheelAcceleration", 0.0)
        self.wheel_torque = rospy.get_param("~wheelTorque", 5.0)

        self.update_rate = rospy.get_param("~updateRate", 100.0)

        # Initialize variables
        self.left_joint_velocity = 0.0
        self.right_joint_velocity = 0.0
        self.last_update_time = rospy.Time.now()
        self.lock = threading.Lock()

        # Initialize publishers and subscribers
        self.cmd_vel_subscriber = rospy.Subscriber(self.command_topic, Twist, self.cmd_vel_callback)
        self.odom_publisher = rospy.Publisher(self.odometry_topic, Odometry, queue_size=10)
        self.transform_broadcaster = tf.TransformBroadcaster()

        # Start the update thread
        self.update_thread = threading.Thread(target=self.update_loop)
        self.update_thread.start()

    def cmd_vel_callback(self, msg: Twist):
        with self.lock:
            linear_x = msg.linear.x
            angular_z = msg.angular.z

            self.left_joint_velocity = linear_x - (angular_z * self.wheel_separation / 2.0)
            self.right_joint_velocity = linear_x + (angular_z * self.wheel_separation / 2.0)

    def update_loop(self):
        rate = rospy.Rate(self.update_rate)
        
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            dt = (current_time - self.last_update_time).to_sec()

            if dt > 0:
                self.publish_odometry(dt)

            self.last_update_time = current_time
            rate.sleep()

    def publish_odometry(self, dt: int):
        x = (self.left_joint_velocity + self.right_joint_velocity) / 2.0 * dt
        theta = (self.right_joint_velocity - self.left_joint_velocity) / self.wheel_separation * dt

        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = self.odometry_frame
        odom_msg.child_frame_id = self.robot_base_frame

        # Set position and orientation (dummy values for illustration)
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = 0  # Assume moving only along x-axis
        odom_msg.pose.pose.position.z = 0

        # Orientation as quaternion (simple integration)
        quaternion = Transform.quaternion_from_euler(0, 0, theta)
        odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w = quaternion

        # Publish odometry
        self.odom_publisher.publish(odom_msg)

        # Send transform
        self.transform_broadcaster.sendTransform(
            (x, 0, 0),
            quaternion,
            rospy.Time.now(),
            self.robot_base_frame,
            self.odometry_frame
        )

if __name__ == '__main__':
    try:
        TTKontrollerDiffDrive()
        rospy.spin() # spin now would handle all related self.run activities
    except rospy.ROSInterruptException:
        pass
