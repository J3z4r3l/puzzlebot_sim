#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry

class Localisation:
    def _init_(self):
        # Initialize
        rospy.init_node("puzz_sim")
        self.loop_rate = rospy.Rate(rospy.get_param("~node_rate",100))

        # Parameters
        self.radius=5.00
        self.wheelbase=19.00

        self.first=True
        ##Publishers
        #self.pub_ = rospy.Publisher('/joint_states', JointState, queue_size=10)
        #rospy.Subscriber('/cmd_vel',Twist,self.twist_callback)
        
        rospy.Subscriber('/pose', PoseStamped, self.callback_pose)
        self.pub_odom = rospy.Publisher('/odom', Odometry, queue_size=10)
        rospy.Subscriber("/wr",Float32,self.odometry_callback)
        rospy.Subscriber("/wl",Float32,self.odometry_callback)

    def callback_pose(self,msg):
        #ponemos lo que vamos a recibir de pose
        pass

    
    # wrap to pi function
    def wrap_to_Pi(self,theta):
        result = np.fmod((theta + np.pi),(2 * np.pi))
        if(result < 0):
            result += 2 * np.pi
        return result - np.pi

    ##NO estoy muy seguro de que esto vaya jaja
    def pose_stamped(self):
        self._pwr=PoseStamped()
        ##Nombre del link de la rueda y sus componentes cercanos
        #self._pwr.header.seq = 1
        self._pwr.header.stamp = rospy.Time.now()
        self._pwr.header.frame_id = "wheel1"
        self._pwr.pose.position.x = 1.5
        self._pwr.pose.position.y = -1.50
        self._pwr.pose.position.z = 0.00
        self._pwr.pose.orientation.x = 00.052
        self._pwr.pose.orientation.y = 00.0972
        self._pwr.pose.orientation.z = 00.00
        self._pwr.pose.orientation.w = 00.00

        self._pwl=PoseStamped()
        ##Nombre del link de la rueda y sus componentes cercanos
        self._pwl.header.seq = 1
        self._pwl.header.stamp = rospy.Time.now()
        self._pwl.header.frame_id = 'wheel2'
        self._pwl.pose.position.x = 00.00
        self._pwl.pose.position.y = 00.00
        self._pwl.pose.position.z = 00.00
        self._pwl.pose.orientation.x = 00.00
        self._pwl.pose.orientation.y = 00.00
        self._pwl.pose.orientation.z = 00.00
        self._pwl.pose.orientation.w = 00.00

    #dos en uno genial!
    def odometry_callback(self, msg_wr, msg_wl):
        # Process data from /wr and /wl topics
        wr = msg_wr.data
        wl = msg_wl.data

        # Calculate linear and angular velocities
        ##No sé si las velocidades ya las teniamos que mandar desde el nodo anterior 
        v = (wr + wl) / 2.0
        omega = (wr - wl) / self.wheelbase

        # Calculate position and orientation
        dt = 1.0 / self.loop_rate.frequency
        x = v * dt
        y = 0.0
        theta = omega * dt

        # Create Odometry message
        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self._pwr.pose.position.x + x
        odom.pose.pose.position.y = self._pwr.pose.position.y + y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = np.sin(theta / 2.0)
        odom.pose.pose.orientation.w = np.cos(theta / 2.0)

        # Fill in the Odometry message with the necessary data
        # ...

        self.pub_odom.publish(odom)

    def twist_callback(self, msg):
        # ...
        self.odometry_callback(self.wr, self.wl)

    def simulate(self):
        self.pose_stamped()

        while not rospy.is_shutdown():
            current_time = rospy.Time.now().to_sec()  # Get current time
            self.pub_pose.publish(self._pwr)

            if self.first:
                self.previous_time = current_time
                self.first = False
            else:
                dt = current_time - self.previous_time  # Calculate time difference
                self.previous_time = current_time

                # Publish /wr and /wl messages
                # ...

                self.odometry_callback(self.wr, self.wl)

            self.loop_rate.sleep()

if __name__=='__main__':
    pendulum=Localisation()
    try:
        pendulum.simulate()
    except rospy.ROSInterruptException:
        pass #Initialise and Setup node