#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
import math

class WheelEncoder:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('wheel_encoder_info')

        # Encoder parameters
        self.ticks_per_revolution = rospy.get_param("~ticks_per_revolution", 360)
        self.encoder_ticks = 0
        self.prev_angle_l = 0.0
        self.prev_angle_r = 0.0
        self.first_run = True

        # ROS publisher for encoder ticks
        self.encoder_pub = rospy.Publisher('/wheel_encoder_ticks', Int32, queue_size=10)

        # ROS subscriber for joint states
        rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)

    def joint_state_callback(self, msg):
        # Find the index of the wheel joint in JointState
        try:
            wheel_index_l = msg.name.index("left_wheel_joint")
            wheel_index_r = msg.name.index("right_wheel_joint")
        except ValueError:
            rospy.logwarn("Wheel joint not found in joint_states")
            return

        # Get the current wheel angle in radians
        current_angle_l = msg.position[wheel_index_l]
        current_angle_r = msg.position[wheel_index_r]


        # Skip calculation on first run to initialize prev_angle
        if self.first_run:
            self.prev_angle_l = current_angle_l
            self.prev_angle_r = current_angle_r
            self.first_run = False
            return

        # Calculate rotation difference
        delta_angle_l = current_angle_l - self.prev_angle_l
        delta_angle_r = current_angle_r - self.prev_angle_r
        self.prev_angle_l = current_angle_l
        self.prev_angle_r = current_angle_r

        # Convert angle difference to encoder ticks
        delta_ticks_l= int((delta_angle_l / (2 * math.pi)) * self.ticks_per_revolution)
        delta_ticks_r= int((delta_angle_r / (2 * math.pi)) * self.ticks_per_revolution)
        self.encoder_ticks_l += delta_ticks_l
        self.encoder_ticks_r += delta_ticks_r

        # Publish encoder ticks
        tick_msg = Int32()
        tick_msg.data = (self.encoder_ticks_l,self.encoder_ticks_r)
        self.encoder_pub.publish(tick_msg)

        rospy.loginfo("Published encoder ticks: %d", self.encoder_ticks)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        encoder_simulator = WheelEncoder()
        encoder_simulator.run()
    except rospy.ROSInterruptException:
        pass