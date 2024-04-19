#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, PoseStamped


class Simulation:
     def __init__(self):
          #Initialize
          rospy.init_node("puzz_sim")
          self.loop_rate = rospy.Rate(rospy.get_param("~node_rate",100))
          
          #Parameters
          self.radius=0.05
          self.wheelbase=0.19
          self.first=True
          self.x_l=0.0
          self.y_l=0.0
          self.z_a=0.0

          ##Publishers and Suscribers
          self.pub_custom_joint_state = rospy.Publisher('/joint_states', JointState, queue_size=10)         
          
          self.pub_js = rospy.Publisher('/joint_states', JointState, queue_size=10)         
          
          self.pub_pose = rospy.Publisher('/pose', PoseStamped, queue_size=10)         
          self.pub_pose_2 = rospy.Publisher('/pose', PoseStamped, queue_size=10)         
          
          rospy.Subscriber('/cmd_vel',Twist,self.twist_callback)
          self.wr=rospy.Publisher("/wr",Float32,queue_size=10)
          self.wl=rospy.Publisher("/wl",Float32,queue_size=10)

     #Parametros para calcular la velocidad de las ruedas
     def twist_callback(self,msg):
          self.x_l = msg.linear.x
          self.y_l = msg.linear.y
          self.z_a = msg.angular.z

     def init_joints(self):
          self.msg=JointState()
          self.msg.header.frame_id= "wheel1"
          self.msg.header.stamp= rospy.Time.now()
          self.msg.name.extend(["wheel_joint1"])
          self.msg.position.extend([0.0])
          self.msg.velocity.extend([0.0])
          self.msg.effort.extend([0.0])
          
          self.msg2=JointState()
          self.msg2.header.frame_id= "wheel2"
          self.msg2.header.stamp= rospy.Time.now()
          self.msg2.name.extend(["wheel_joint2"])
          self.msg2.position.extend([0.0])
          self.msg2.velocity.extend([0.0])
          self.msg2.effort.extend([0.0])
          
     
   
    #wrap to pi function
     def wrap_to_Pi(self,theta):
           result = np.fmod((theta + np.pi),(2 * np.pi))
           if(result < 0):
                result += 2 * np.pi
           return result - np.pi
     
     
     def pose_stamped(self):
          self._robot=PoseStamped()
          ##Nombre del link de la rueda y sus componentes cercanos
          self._robot.header.seq = 1
          self._robot.header.stamp = rospy.Time.now()
          self._robot.header.frame_id = "wheel1"
          self._robot.pose.position.x = 0.0
          self._robot.pose.position.y = 0.0
          self._robot.pose.position.z = 0.00
          self._robot.pose.orientation.x = 00.00
          self._robot.pose.orientation.y = 00.00
          self._robot.pose.orientation.z = 00.00

          
     
     def simulate(self):
          self.pose_stamped()
          self.init_joints()

          theta=0.0
          x_dot=0.0
          y_dot=0.0
          x=0.0
          y=0.0


          while not rospy.is_shutdown():
              current_time = rospy.Time.now().to_sec()  # Get current time
              #self.pub_pose.publish(self._robot)
              if self.first:
                  self.previous_time = current_time
                  self.first = False
              else:
                  #calculamos el diferencial de tiempo 
                  dt = current_time - self.previous_time  # Calculate time difference
                  self.previous_time = current_time
                  
                  #obteniendo las velocidades de Wl y Wr
                  wr_1 = (2.0*(self.x_l) + (self.wheelbase *self.z_a))/(2.0*self.radius)
                  wl_1= (2.0*(self.x_l) - (self.wheelbase * self.z_a))/(2.0*self.radius)
                  rospy.loginfo(wl_1)
                  #Publicamos velocidades
                  self.wr.publish(wr_1)
                  self.wl.publish(wl_1)
                  
                  #Obtenmos la pos y velocidad lineal como angular de las velocidades podemos integrar con metodo de euler y obtener la posicion 
                  theta_dot=self.wrap_to_Pi(self.radius*((wr_1-wl_1)/self.wheelbase))
                  theta+=theta_dot*dt 
                  
                  x+=x_dot*dt 
                  x_dot=self.radius*((wr_1+wl_1)/2)*np.cos(theta) #vel
                  ##rospy.loginfo(x_dot)
                  y +=y_dot*dt 
                  y_dot=self.radius*((wr_1+wl_1)/2)*np.sin(theta) #vel
                  

                  ##Publicamos las poses
                  self._robot.pose.position.x = x
                  self._robot.pose.position.y = y
                  self._robot.pose.orientation.z= theta

                  

                  self.msg.header.stamp = rospy.Time.now()
                  self.msg.position[0] =  x
                  self.msg.velocity[0] = wl_1
                  
                  self.msg2.header.stamp = rospy.Time.now()
                  self.msg2.position[0] = x
                  self.msg2.velocity[0] = wr_1
                  
                  self.pub_custom_joint_state.publish(self.msg)

                  self.pub_js.publish(self.msg2)

                  self.pub_pose.publish(self._robot)
                  
     
     
                  self.loop_rate.sleep()

if __name__=='__main__':
    pendulum=Simulation()
    try:
         pendulum.simulate()
         
                    
    except rospy.ROSInterruptException:
        pass #Initialise and Setup node

##Linea terminal modificar el cmd_vel
###rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: 1.0}, angular: {z: 1.0}}'
