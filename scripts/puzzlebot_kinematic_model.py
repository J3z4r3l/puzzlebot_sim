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
          self.radius=5.00
          self.wheelbase=19.00
          self.first=True
          self.x_l=0.0
          self.y_l=0.0
          self.z_a=0.0

          ##Publishers and Suscribers
          self.pub_pose = rospy.Publisher('/pose', PoseStamped, queue_size=10)         
          rospy.Subscriber('/cmd_vel',Twist,self.twist_callback)
          self.wr=rospy.Publisher("/wr",Float32,queue_size=10)
          self.wl=rospy.Publisher("/wl",Float32,queue_size=10)

     #Parametros para calcular la velocidad de las ruedas
     def twist_callback(self,msg):
          self.x_l = msg.linear.x
          self.y_l = msg.linear.y
          self.z_a = msg.angular.z

    
   
    #wrap to pi function
     def wrap_to_Pi(self,theta):
           result = np.fmod((theta + np.pi),(2 * np.pi))
           if(result < 0):
                result += 2 * np.pi
           return result - np.pi
     
     
     def pose_stamped(self):
          self._pwr=PoseStamped()
          ##Nombre del link de la rueda y sus componentes cercanos
          self._pwr.header.seq = 1
          self._pwr.header.stamp = rospy.Time.now()
          self._pwr.header.frame_id = "wheel1"
          self._pwr.pose.position.x = 1.5
          self._pwr.pose.position.y = -1.50
          self._pwr.pose.position.z = 0.00
          self._pwr.pose.orientation.x = 00.052
          self._pwr.pose.orientation.y = 00.0972
          self._pwr.pose.orientation.z = 00.00
          
          self._pwl=PoseStamped()
        ##Nombre del link de la rueda y sus componentes cercanos
          self._pwl.header.seq = 1
          self._pwl.header.stamp = rospy.Time.now()
          self._pwl.header.frame_id = 'wheel2'
          self._pwl.pose.position.x = 00.00
          self._pwl.pose.position.y = 00.00
          self._pwl.pose.position.z = 00.00
          self._pwl.pose.orientation.x = 00.052
          self._pwl.pose.orientation.y = 00.0972
          self._pwl.pose.orientation.z = 00.00
          
     
     def simulate(self):
          self.pose_stamped()
          theta=0.0
          x_dot=0.0
          y_dot=0.0


          while not rospy.is_shutdown():
              current_time = rospy.Time.now().to_sec()  # Get current time
              #self.pub_pose.publish(self._pwr)
              if self.first:
                  self.previous_time = current_time
                  self.first = False
              else:
                  #calculamos el diferencial de tiempo 
                  dt = current_time - self.previous_time  # Calculate time difference
                  self.previous_time = current_time
                  
                  #obteniendo las velocidades de Wl y Wr
                  wr_1 = (2.0*(self.x_l) + (self.wheelbase * self.z_a))/2.0
                  wl_1= (2.0*(self.x_l) - (self.wheelbase * self.z_a))/2.0
                  #Publicamos velocidades
                  #rospy.loginfo(self.x_l)
                  self.wr.publish(wr_1)
                  self.wl.publish(wl_1)
                  
                  #Obtenmos la pos y velocidad lineal como angular de las velocidades podemos integrar con metodo de euler y obtener la posicion 
                  #z normalizando el angulo 
                  theta_dot=self.wrap_to_Pi(self.radius*((wr_1-wl_1)/self.wheelbase))
                  theta=theta_dot*dt 
                  rospy.loginfo(theta)
                  #x
                  x=x_dot*dt #pos
                  x_dot=self.radius*((wr_1+wl_1)/2)*np.cos(theta) #vel
                  rospy.loginfo(x_dot)
                  #y
                  y=y_dot*dt #pos
                  y_dot=self.radius*((wr_1+wl_1)/2)*np.sin(theta) #vel
                  rospy.loginfo(y_dot)
                  

                  ##Publicamos las poses
                  self._pwl.pose.position.x = x
                  self._pwl.pose.position.y = y
                  self._pwl.pose.orientation.w= theta
          

                  self.pub_pose.publish(self._pwl)
                  
     
     
                  self.loop_rate.sleep()

if __name__=='__main__':
    pendulum=Simulation()
    try:
         pendulum.simulate()
         
                    
    except rospy.ROSInterruptException:
        pass #Initialise and Setup node

##Linea terminal modificar el cmd_vel
###rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: 1.0}, angular: {z: 1.0}}'
