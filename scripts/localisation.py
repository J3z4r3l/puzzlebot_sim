#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import tf.transformations as tf

class LocalizationNode:
    def __init__(self):
        rospy.init_node("localisation")
        self.rate = rospy.Rate(100) 
        self.base_time = rospy.Time.now()
        self.wheelbase = 0.005  
        self.radius = 0.05 
        self.pose = PoseStamped()
        self.pose.header.frame_id = "odom"
        self.wr_speed = 0.0
        self.wl_speed = 0.0
        #self.pose.child_frame_id = "base_link"
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        rospy.Subscriber("/wr", Float32, self.wr_callback)
        rospy.Subscriber("/wl", Float32, self.wl_callback)

    def wr_callback(self, msg):
        self.wr_speed = msg.data

    def wl_callback(self, msg):
        self.wl_speed = msg.data

    def calculate_odometry(self):
        current_time = rospy.Time.now()
        dt = (current_time - self.base_time).to_sec()
        self.base_time = current_time

        wr = self.wr_speed
        wl = self.wl_speed

        # Calculate robot's linear and angular speed
        v = self.radius * (wr + wl) / 2.0
        w = self.radius * (wr - wl) / self.wheelbase

        # Update robot's pose
        self.pose.pose.position.x += v * np.cos(self.pose.pose.orientation.z) * dt
        self.pose.pose.position.y += v * np.sin(self.pose.pose.orientation.z) * dt
        self.pose.pose.orientation.z += w * dt

        # Create and publish Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose = self.pose.pose
        self.odom_pub.publish(odom_msg)

    def run(self):
        while not rospy.is_shutdown():
            self.calculate_odometry()
            self.rate.sleep()

if __name__ == "__main__":
    try:
        node = LocalizationNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
