#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
import threading
import tf.transformations as Transform
from std_msgs.msg import Int32MultiArray

import math

class TTKontrollerDiffDrive:
    def __init__(self):
        rospy.init_node('tt_kontroller_diff_drive', anonymous=True)

        # Parameters
        self.command_topic = rospy.get_param("~commandTopic", "cmd_vel")
        self.odometry_topic = rospy.get_param("~odometryTopic", "odom")
        self.wheel_encoder_topic = rospy.get_param("~encoderTopic", "wheel_encoder_ticks")
        
        self.odometry_frame = rospy.get_param("~odometryFrame", "odom")
        self.robot_base_frame = rospy.get_param("~robotBaseFrame", "base_link")

        self.wheel_separation = rospy.get_param("~wheelSeparation", 0.34)
        self.wheel_diameter = rospy.get_param("~wheelDiameter", 0.15)
        self.wheel_accel = rospy.get_param("~wheelAcceleration", 0.0)
        self.wheel_torque = rospy.get_param("~wheelTorque", 5.0)

        self.update_rate = rospy.get_param("~updateRate", 100.0)

        self.q = 360  # Encoder ticks per revolution
        self.c = 0.3  # Wheel circumference in meters
        self.R_w = 0.15  # Half the wheel separation (in meters)

        # Initialize variables
        self.left_ticks_prev = 0
        self.right_ticks_prev = 0
        self.left_ticks = 0
        self.right_ticks = 0

        self.left_joint_velocity = 0.0
        self.right_joint_velocity = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_update_time = rospy.Time.now()
        self.lock = threading.Lock()

        # Initialize publishers and subscribers
        self.cmd_vel_subscriber = rospy.Subscriber(self.command_topic, Twist, self.cmd_vel_callback)
        self.odom_publisher = rospy.Publisher(self.odometry_topic, Odometry, queue_size=10)
        self.encoder_subscriber = rospy.Subscriber(self.wheel_encoder_topic, Int32MultiArray, self.encoder_callback)

        # Start the update thread
        self.update_thread = threading.Thread(target=self.update_loop)
        self.update_thread.start()

    def cmd_vel_callback(self, msg: Twist):
        with self.lock:
            linear_x = msg.linear.x
            angular_z = msg.angular.z

            # Calculate wheel velocities from linear and angular velocities
            self.left_joint_velocity = linear_x - (angular_z * self.wheel_separation / 2.0)
            self.right_joint_velocity = linear_x + (angular_z * self.wheel_separation / 2.0)

    def encoder_callback(self, msg: Int32MultiArray):
        self.left_ticks = msg.data[0]
        self.right_ticks = msg.data[1]

    def update_loop(self):
        rate = rospy.Rate(10)  # Update at 10 Hz

        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            dt = (current_time - self.last_update_time).to_sec()

            if dt > 0:
                self.update_odometry(dt)

            self.last_update_time = current_time
            rate.sleep()

    def update_odometry(self, dt):
        # Calculate distance traveled by each wheel
        dl = ((self.left_ticks - self.left_ticks_prev) / self.q) * self.c
        dr = ((self.right_ticks - self.right_ticks_prev) / self.q) * self.c

        # Store previous ticks for next iteration
        self.left_ticks_prev = self.left_ticks
        self.right_ticks_prev = self.right_ticks

        # Calculate change in orientation (theta) and distance traveled
        delta_theta = (dr - dl) / (2 * self.R_w)
        d = (dl + dr) / 2

        # Update robot's position and orientation
        self.x += d * math.cos(self.theta + delta_theta / 2)
        self.y += d * math.sin(self.theta + delta_theta / 2)
        self.theta += delta_theta

        # Publish odometry data
        self.publish_odometry()

    def publish_odometry(self):
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = self.odometry_frame
        odom_msg.child_frame_id = self.robot_base_frame

        # Set position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y

        # Set orientation as quaternion
        quaternion = Transform.quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x = quaternion[0]
        odom_msg.pose.pose.orientation.y = quaternion[1]
        odom_msg.pose.pose.orientation.z = quaternion[2]
        odom_msg.pose.pose.orientation.w = quaternion[3]

        # Publish odometry message
        self.odom_publisher.publish(odom_msg)

if __name__ == '__main__':
    try:
        TTKontrollerDiffDrive()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass