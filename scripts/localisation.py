#!/usr/bin/env python
# This is a ROS node that subscribes to the left and right wheel speeds
# and publishes the robot's odometry.

import rospy 
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from tf import TransformBroadcaster

class LocalizationNode:
    def _init_(self):
        # Initialize the ROS node named "localisation"
        rospy.init_node("localisation")

        # Set the loop rate to 100 Hz
        self.rate = rospy.Rate(100) 

        # Save the current time for calculating the time delta
        self.base_time = rospy.Time.now()

        # Define the wheelbase and radius of the robot
        self.wheelbase = 0.191 
        self.radius = 0.05 

        # Initialize the robot's pose
        self.pose = PoseStamped()
        self.pose.header.frame_id = "odom"
        self.pose.child_frame_id = "base_link"

        # Create a publisher for the robot's odometry
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)

        # Create a TransformBroadcaster
        self.odom_broadcaster = TransformBroadcaster()

        # Subscribe to the left and right wheel speeds
        rospy.Subscriber("/wr", Float64, self.wr_callback)
        rospy.Subscriber("/wl", Float64, self.wl_callback)

        self.first = True

    def wr_callback(self, msg):
        # Callback function for the right wheel speed
        self.wr_speed = msg.data

    def wl_callback(self, msg):
        # Callback function for the left wheel speed
        self.wl_speed = msg.data

    def calculate_odometry(self):
        if self.first:
            self.previous_time = rospy.Time.now()
            self.first = False
        else:
            # Calculate the robot's odometry
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

            # Publish the transform from odometry to base_link
            self.odom_broadcaster.sendTransform(
                (self.pose.pose.position.x, self.pose.pose.position.y, self.pose.pose.position.z),
                (self.pose.pose.orientation.x, self.pose.pose.orientation.y, self.pose.pose.orientation.z, self.pose.pose.orientation.w),
                current_time,
                "base_link",
                "odom"
            )

    def run(self):
        # Run the node
        while not rospy.is_shutdown():
            self.calculate_odometry()
            self.rate.sleep()

if __name__ == "__main__":
    try:
        node = LocalizationNode()
        node.run()
    except rospy.ROSInterruptException:
        pass