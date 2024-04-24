#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import numpy as np
from nav_msgs.msg import Odometry


class Controller:
    def __init__(self):
         # PID distancia
        self.kp_dist = 0.09  
        self.ki_dist = 0.0
        self.kd_dist = 0.0  
        
        self.error_dist=0.0
        self.integral_dist=0.0
        self.prev_error_dist=0.0
        self.velocidad_l=0

        # PID angulo
        self.kp_ang = 0.2   
        self.ki_ang = 0.03  
        self.kd_ang = 0.00  

        self.error_ang=0.0
        self.velocidad_a=0
        self.integral_ang=0.0
        self.prev_error_ang=0.0

        # Variables de estado
        self.x = 0.0
        self.y = 0.0
        self.ori_w = 0.0
        #Lista de puntos 
        self.index = 0
        self.x_list = [1, 1, 0, 0, 0]
        self.y_list = [0, 1, 1, 0, 0]
        self.first=True

        # Inicializar nodos
        rospy.init_node("controller")
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.pose_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.msg = Twist()
        self.rate = rospy.Rate(10)

    
    def odom_callback(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.ori_w = self.wrap_to_Pi(data.pose.pose.orientation.z)

    def wrap_to_Pi(self,theta):
        result = np.fmod((theta + np.pi),(2 * np.pi))
        if(result < 0):
                result += 2 * np.pi
        return result - np.pi
    
    def error_a_l(self,index):
        error_ang = np.arctan2(self.y_list[index] - self.y, self.x_list[index] - self.x) - self.ori_w
        error_dist = np.sqrt((self.x_list[index] - self.x) ** 2 + (self.y_list[index] - self.y) ** 2)
        return error_ang, error_dist

   
    def pid_controller(self,dt,error_ang,error_dist,prev_error_dist,prev_error_ang): 
        # Controlador PID para la distancia
        self.integral_dist += error_dist * dt
        derivative_dist = (error_dist - prev_error_dist) / dt
        self.velocidad_l = self.kp_dist * self.error_dist + self.ki_dist * self.integral_dist + self.kd_dist * derivative_dist
        # Controlador PID para el ang
        self.integral_ang += error_ang * dt
        derivative_ang = (error_ang - prev_error_ang) / dt
        self.velocidad_a = self.kp_ang * error_ang + self.ki_ang * self.integral_ang + self.kd_ang * derivative_ang



    def run(self):
        while not rospy.is_shutdown():
            if self.first:
                self.current_time = rospy.get_time() 
                self.previous_time = rospy.get_time()
                self.first = False
            else:
                self.current_time = rospy.get_time() 
                dt = (self.current_time - self.previous_time) #get dt
                self.previous_time = self.current_time
                self.error_ang, self.error_dist = self.error(self.index)
               
                # PID controller 
                self.velocidad_l, self.velocidad_a = self.pid_controller(self.error_ang, self.error_dist,self.prev_error_dist,self.prev_error_ang)
                self.prev_error_dist = self.error_dist
                self.prev_error_ang = self.error_ang
                
                if abs(self.error_ang) < 0.6:
                    # Si el error angular es pequeno, establece la velocidad angular en cero
                    self.velocidad_a = 0
                    self.error_ang = 0

                # Pausa para mantener la frecuencia de publica
                if self.error_dist < 0.12:
                    self.error_dist = 0

                if abs(self.error_ang) < 0.5 and self.error_dist == 0 and self.index < len(self.x_list)-1:
                    self.index += 1

                if self.index == len(self.x_list)-1:
                    self.error_dist = 0
                    self.error_ang = 0
                    self.velocidad_l = 0
                    self.velocidad_a = 0
                else:
                    # Publicar las velocidades del /cmd_vel solo si no hemos alcanzado el final de la lista
                    self.msg.angular.z = self.velocidad_a
                    self.msg.linear.x = self.velocidad_l

                    print_info = "%3f | %3f | %3f | %3f " %(self.index, self.velocidad_l, self.error_ang, self.error_dist)
                    rospy.loginfo(print_info)
                    self.pose_pub.publish(self.msg)
                    self.rate.sleep()



if __name__== "__main__":
    controller = Controller()
    try:
        controller.run()
    except rospy.ROSInterruptException:
        None
