#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import numpy as np
#from mini_c2.msg import input_point
from nav_msgs.msg import Odometry

class PointGen:
    def __init__(self):
        self.x = []
        self.y = []
        self.index = int

class Controller:
    def __init__(self):
        # Configurar parametros del controlador lineal y angular 
        self.kp_l = 0.09
        self.ki_l = 0.0
        self.kd_l = 0.0

        self.kp_ang = 0.4
        self.ki_ang = 0.03
        self.kd_ang = 0.0

        self.error_sum_l = 0.0
        self.error_prev_l = 0.0
        self.error_diff_l = 0.0
        self.error_sum_ang = 0.0
        self.error_prev_ang = 0.0
        self.error_diff_ang = 0.0
        self.error_dist=0.0
        self.error_ang=0.0
        self.first=True
        self.velocidad_l=0
        self.velocidad_ang=0
        self.controlador_vl=0.0
        self.controlador_va=0.0        
        self.last_vl=0.0
        self.last_va=0.0
        self.i=0

        #Inicializar nodos
        rospy.init_node("controller")
        rospy.Subscriber("/set_point",PointGen,self.target_callback)
        rospy.Subscriber("/odom",Odometry,self.odom_callback)   
        self.pose_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)

        self.msg = Twist()
        self.msg.linear.x = 0
        self.msg.linear.y = 0
        self.msg.linear.z = 0
        self.msg.angular.x = 0
        self.msg.angular.y = 0
        self.msg.angular.z = 0

        self.rate = rospy.Rate(10)

    def odom_callback(self, data):
        self.x = data.pose.pose.pos_x
        self.y = data.pose.pose.pos_y
        self.z = data.pose.pose.pos_z
        self.ori_x = data.pose.pose.ori.x
        self.ori_y = data.pose.pose.ori.y
        self.ori_z = data.pose.pose.ori.z
        self.ori_w = data.pose.pose.ori.w

    def set_point_callback(self, msg):
        self.x_point = msg.x
        self.y_point = msg.y
        self.position = msg.index

    def stop(self):
        print("Stopping")
        self.msg.linear.x = 0
        self.msg.linear.y = 0
        self.msg.linear.z = 0
        self.msg.angular.x = 0
        self.msg.angular.y = 0
        self.msg.angular.z = 0
        self.pose_pub.publish(self.msg)

    def run(self):
        while not rospy.is_shutdown():
            if self.first:
                self.current_time = rospy.get_time() 
                self.previous_time = rospy.get_time()
                self.first = False
            else:
                self.current_time = rospy.get_time() 
                dt = (self.current_time - self.previous_time)
                self.previous_time = self.current_time
                
                def wrap_to_Pi(self,theta):
                    result = np.fmod((theta + np.pi),(2 * np.pi))
                    if(result < 0):
                            result += 2 * np.pi
                    return result - np.pi

                #Calculo de errores 
                self.error_ang = np.atan2(self.x_target,self.y_target) - wrap_to_Pi(self.ori_w)
                self.error_dist = np.sqrt(np.power((self.x_target - self.x),2)+(np.power((self.y_target - self.y),2)))
                
                #Controlador lineal  
                self.error_sum_l += self.error_dist * dt
                self.error_diff_l = 0#(self.error_dist - self.error_prev_l) / dt
                self.error_prev_l = self.error_dist
                self.controlador_vl= self.kp_l * self.error_dist + self.ki_l * self.error_sum_l + self.kd_l * self.error_diff_l
                self.velocidad_l =self.last_vl +((self.controlador_vl-self.last_vl))
                self.last_vl=self.velocidad_l

                #controlador angular
                self.error_sum_ang += self.error_ang * dt
                self.error_diff_ang = 0#(self.error_ang - self.error_prev_ang) / dt
                self.error_prev_ang = self.error_ang
                self.controlador_va =self.kp_ang * self.error_ang + self.ki_ang * self.error_sum_ang + self.kd_ang * self.error_diff_ang
                self.velocidad_ang = self.last_va +((self.controlador_va-self.last_va))
                self.last_va=self.velocidad_ang

                #Cambio de posiciones
                if self.error_dist < 0.1:
                    self.error_dist=0
                if self.error_ang < 0.1 and self.error_ang > -0.1:
                    self.error_ang=0
                if self.error_ang==0 and self.error_dist==0 and self.i<self.position:
                    self.x_target = self.x_point[self.i]
                    self.y_target = self.y_point[self.i]
                    self.i+=1
                if self.i==self.position:
                    self.error_ed=0
                    self.error_eang=0 
                                
                #Publicar las posiciones
                self.msg.linear.x = self.velocidad_l
                self.msg.angular.z = self.velocidad_ang
                print_info = "%3f | %3f  " %(self.velocidad_l,self.velocidad_ang)
                rospy.loginfo(print_info)
                self.pose_pub.publish(self.msg)
                self.rate.sleep()

if __name__ == "__main__":
    controller = Controller()
    try:
        controller.run()
    except rospy.ROSInterruptException:
        None