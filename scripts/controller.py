#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import numpy as np
#from mini_c2.msg import input_point
from nav_msgs.msg import Odometry


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

        ##Variables set_point 
        self.x_list = [1, 2, 3, 4, 0]
        self.y_list = [0, 0, 0, 0, 0]
        self.index = 0
        self.y=0.0
        self.x=0.0
        self.ori_w=0.0

        #Inicializar nodos
        rospy.init_node("controller")
        rospy.Subscriber("/odom",Odometry,self.odom_callback)   
        self.pose_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)

        self.msg = Twist()
        

        self.rate = rospy.Rate(10)
    def odom_callback(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.ori_w = data.pose.pose.orientation.w

    
    def wrap_to_Pi(self,theta):
        result = np.fmod((theta + np.pi),(2 * np.pi))
        if(result < 0):
                result += 2 * np.pi
        return result - np.pi
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
                self.error_ang = np.arctan2(self.y_list[self.index]-self.y, self.x_list[self.index]-self.x) - (self.ori_w-1)
                self.error_dist = np.sqrt(np.square(self.x_list[self.index]-self.x) + np.square(self.y_list[self.index]-self.y))
                rospy.loginfo(self.error_dist)
                
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

                
                #Cambio de posiciones esto esta bien 
                if self.error_dist < 0.15:
                    self.error_dist=0
                if self.error_ang < 0.15 and self.error_ang > -0.15:
                    self.error_ang=0
                if self.error_ang==0 and self.error_dist==0 and self.index<len(self.x_list)-1:
                    #self.index+=1
                    self.index+=1
                    
                if self.index==len(self.x_list)-1:
                    self.error_ed=0
                    self.error_eang=0
                    self.velocidad_l=0
                    self.velocidad_ang=0

 
                else:
    #Publicar las velocidades del /cmd_vel solo si no hemos alcanzado el final de la lista
                    self.msg.angular.z = self.velocidad_ang
                    self.msg.linear.x = self.velocidad_l
                    rospy.loginfo(self.index)
                    
                    print_info = "%3f | %3f  " %(self.velocidad_l,self.velocidad_ang)
                    #rospy.loginfo(print_info)
                    self.pose_pub.publish(self.msg)
                    self.rate.sleep()

if __name__ == "__main__":
    controller = Controller()
    try:
        controller.run()
    except rospy.ROSInterruptException:
        None