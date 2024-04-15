#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, PoseStamped


class Simulation:
     def __init__(self):
          #Initialize
          rospy.init_node("sim")
          self.loop_rate = rospy.Rate(rospy.get_param("~node_rate",100))
         
          #Parameters
          self.radius=5.00
          self.wheelbase=19.00



          self.first=True
          ##Publishers 
          self.pub_ = rospy.Publisher('/joint_states', JointState, queue_size=10)         
          self.pub_pose = rospy.Publisher('/pose', PoseStamped, queue_size=10)         
          self.vel_sub = rospy.Subscriber('/cmd_vel',Twist,self.twist_callback)

        

    #wrap to pi function
     def twist_callback(self,msg):
          self.msg_t = Twist()
          self.msg_t.linear.x = 0.0
          self.msg_t.linear.y = 0.0
          self.msg_t.linear.z = 0.0
          self.msg_t.angular.x = 0.0
          self.msg_t.angular.y = 0.0
          self.msg_t.angular.z = 0.0

    
   
    #wrap to pi function
     def wrap_to_Pi(self,theta):
           result = np.fmod((theta + np.pi),(2 * np.pi))
           if(result < 0):
                result += 2 * np.pi
           return result - np.pi
     
     ##Ponemos la posicion de nuestros nodos 
     def init_joints(self):
          self.msg=JointState()
          self.msg.header.frame_id= "link1"
          self.msg.header.stamp= rospy.Time.now()
          self.msg.name.extend(["joint2"])
          self.msg.position.extend([0.0])
          self.msg.velocity.extend([0.0])
          self.msg.effort.extend([0.0])
          

     
     def simulate(self):
          self.init_joints()

          while not rospy.is_shutdown():
              current_time = rospy.Time.now().to_sec()  # Get current time

              if self.first:
                  self.previous_time = current_time
                  self.first = False
              else:
                  dt = current_time - self.previous_time  # Calculate time difference
                  self.previous_time = current_time
                  self.x1 += self.x2 * dt
                  self.x2_dot = (1 / self.j) * (self.tao - self.m * self.g * self.a * np.cos(self.x1) - self.k * self.x2)
                  self.x2 += self.x2_dot*dt

                  # Update message
                  self.msg.header.stamp = rospy.Time.now()
                  self.msg.position[0] = self.wrap_to_Pi(self.x1)
                  self.msg.velocity[0] = self.x2
                  self.pub_custom_joint_state.publish(self.msg)
                  self.loop_rate.sleep()

if __name__=='__main__':
    pendulum=Simulation()
    try:
         ##pendulum.simulate()
         pass
                    
    except rospy.ROSInterruptException:
        pass #Initialise and Setup node