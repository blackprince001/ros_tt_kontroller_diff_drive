import rospy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
import tf
import tf.transformations

 
class TTKontrollerDiffDrive:
    def __init__(self):
        rospy.init_node("tt_kontroller_diff_drive")
        
        self.cmd_vel_sub = rospy.Subscriber("cmd_vel", 10, self.cmd_vel_callback)
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.tf_broadcaster = tf.TransformBroadcaster()
 
        # we are going to take more parameters later
        self.publish_cmd = rospy.get_param("~publish_cmd", False)

        if self.publish_cmd:
            self.publish_cmd_pub = rospy.Publisher("publish_cmd", TwistStamped, queue_size=10)

        self.current_pose = [0.0, 0.0, 0.0]
        self.rate = rospy.Rate(10) # use parameters for this
 
    def cmd_vel_callback(self, msg):
        # Process the velocity command
        rospy.loginfo(f"Received cmd_vel: linear {msg.linear.x} angular {msg.angular.z}")

        # Compute odometry from hardware feedback
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"
        # Fill in the odom_msg fields
 
        # Publish odometry
        self.odom_pub.publish(odom_msg)
 
        # Publish tf transform
        self.tf_broadcaster.sendTransform(
            (self.current_pose[0], self.current_pose[1], 0),
            tf.transformations.quaternion_from_euler(0, 0, self.current_pose[2]),
            rospy.Time.now(),
            "base_footprint",
            "odom"
        )

        if self.publish_cmd:
            publish_cmd_msg = TwistStamped()
            publish_cmd_msg.header.stamp = rospy.Time.now()
            publish_cmd_msg.twist = msg

            self.publish_cmd_pub.publish(publish_cmd_msg)

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
 

if __name__ == "__main__":
    node = TTKontrollerDiffDrive()
    node.run()