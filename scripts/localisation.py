#!/usr/bin/env python
import rospy 
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf import TransformBroadcaster
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Float64MultiArray


class LocalizationNode:
    def __init__(self):
        # Initialize the robot's pose
        rospy.init_node("localisation")
        self.rate = rospy.Rate(100) 
        self.previous_time = rospy.Time.now()

        # Define the wheelbase and radius of the robot
        self.first = True
        self.wheelbase = 0.19 
        self.radius = 0.05 
        self.wr_speed = 0.0
        self.wl_speed = 0.0
        self.theta = 0.0
        self.x=0.0
        self.y=0.0
        self.odom_msg = Odometry()
        self.snt_tnf=TransformBroadcaster()
        self.pose_robot=PoseStamped()
        self.H_matrix=[]
        self.covariance_matrix=[[0, 0, 0],
                                [0, 0, 0],
                                [0, 0,  0]]
        self.Qk_matrix=[]
        

        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        rospy.Subscriber("/wr", Float32, self.wr_callback)
        rospy.Subscriber("/wl", Float32, self.wl_callback)
        rospy.Subscriber("/pose", PoseStamped, self.pose_stamped)
        rospy.Subscriber("/covariance", Float64MultiArray, self.matrix_callback)
        
         
    def wr_callback(self, v_r):
        self.wr_speed = v_r.data

    def wl_callback(self, v_l):
        self.wl_speed = v_l.data
    
    def pose_stamped(self, msg):
    # Update robot's pose
        self.pose_robot = msg
    
    def matrix_callback(self,msg):
        self.covariance_mtx=msg.data
        
    def get_positon(self,wr,wl,dt):
        vel=self.radius*(wr+wl)/2
        w=self.radius*(wr-wl)/self.wheelbase
        x_dot= vel*np.cos(self.theta) #vel
        y_dot=vel*np.sin(self.theta) #vel
        theta_dot=w
        #Linealizado
        #x_dot= -vel*np.sin(self.theta) #vel
        #y_dot=vel*np.cos(self.theta) #vel
        #theta_dot=w
        
        self.x += x_dot*dt
        print(w)
        self.y += y_dot*dt
        self.theta += theta_dot*dt
        self.wrap_to_Pi(self.theta)
        return    
      
    def get_odometry(self,current_time,x,y,theta,covarianza):
        self.odom_msg.header.stamp = current_time
        self.odom_msg.header.frame_id = "odom" #or odom
        self.odom_msg.child_frame_id = "base_link"
        self.odom_msg.pose.pose.position.x = x
        self.odom_msg.pose.pose.position.y = y
        quaternion=quaternion_from_euler(0,0,theta)
        self.odom_msg.pose.pose.orientation.x = quaternion[0]
        self.odom_msg.pose.pose.orientation.y = quaternion[1]
        self.odom_msg.pose.pose.orientation.z = quaternion[2]
        self.odom_msg.pose.pose.orientation.w = quaternion[3]
        #
        self.odom_msg.pose.covariance[0] = covarianza[0][0]#0.998
        self.odom_msg.pose.covariance[1] = covarianza[0][1]#0.0207
        self.odom_msg.pose.covariance[5] = covarianza[0][2]#0.0180
        self.odom_msg.pose.covariance[6] = covarianza[1][0]#0.0207
        self.odom_msg.pose.covariance[7] = covarianza[1][1]#1.0040
        self.odom_msg.pose.covariance[11]= covarianza[1][2]#0.0399
        self.odom_msg.pose.covariance[30]= covarianza[2][0]#0.0180
        self.odom_msg.pose.covariance[31]= covarianza[2][1]#0.0399
        self.odom_msg.pose.covariance[35]= covarianza[2][2]#0.4000
        #
        return self.odom_msg

    def transform(self,odom):
        tnf=TransformStamped()
        tnf.header.stamp=rospy.Time.now()
        tnf.header.frame_id= "odom"
        tnf.child_frame_id="base_lonk"
        tnf.transform.translation.x = odom.pose.pose.position.x
        tnf.transform.translation.y = odom.pose.pose.position.y
        tnf.transform.translation.z = 0.0
        #rotation
        tnf.transform.rotation.x = odom.pose.pose.orientation.x
        tnf.transform.rotation.y = odom.pose.pose.orientation.y
        tnf.transform.rotation.z = odom.pose.pose.orientation.z
        tnf.transform.rotation.w = odom.pose.pose.orientation.w
        self.snt_tnf.sendTransform(
        (tnf.transform.translation.x, tnf.transform.translation.y, tnf.transform.translation.z),
        (tnf.transform.rotation.x, tnf.transform.rotation.y, tnf.transform.rotation.z, tnf.transform.rotation.w),
        rospy.Time.now(),
        "base_link",
        tnf.header.frame_id
    )

    def wrap_to_Pi(self,theta):
        result = np.fmod((theta + np.pi),(2 * np.pi))
        if(result < 0):
                result += 2 * np.pi
        return result - np.pi
    
    #functions to get the covariance
    def get_positon(self,wr,wl,dt):
        vel=self.radius*(wr+wl)/2
        w=self.radius*(wr-wl)/self.wheelbase
        x_dot= vel*np.cos(self.theta) #vel
        y_dot=vel*np.sin(self.theta) #vel
        theta_dot=w
        self.odom_msg.twist.twist.linear.x=vel
        self.odom_msg.twist.twist.angular.z= w
        
        #Linealizado
        #x_dot= -vel*np.sin(self.theta) #vel
        #y_dot=vel*np.cos(self.theta) #vel
        #theta_dot=w
        self.x += x_dot*dt
        self.y += y_dot*dt
        self.theta += theta_dot*dt
        self.wrap_to_Pi(self.theta)
        return    
    
    def multiply_matrices(self,A, B):
        if len(A[0]) != len(B):
            raise ValueError("El numero de columnas de la matriz A debe ser igual al numero de filas de la matriz B")
        new_matrix = [[0 for _ in range(len(B[0]))] for _ in range(len(A))]
        for i in range(len(A)):
            for j in range(len(B[0])):
                for k in range(len(B)):
                    new_matrix[i][j] += A[i][k] * B[k][j]
        return new_matrix
   
    def add_matrices(self,A, B):
        if len(A) != len(B) or len(A[0]) != len(B[0]):
            raise ValueError("Las matrices deben tener la misma dimension para poder sumarse")
        new_matrix = [[0 for _ in range(len(A[0]))] for _ in range(len(A))]
        for i in range(len(A)):
            for j in range(len(A[0])):
                new_matrix[i][j] = A[i][j] + B[i][j]
        return new_matrix

    def transpose_matrix(self,matrix):
        rows = len(matrix)
        cols = len(matrix[0])
        transposed = [[0 for _ in range(rows)] for _ in range(cols)]
        for i in range(rows):
            for j in range(cols):
                transposed[j][i] = matrix[i][j]

        return transposed
    
    def multiply_3x2_by_2x2(self,A, B):
        if len(A[0]) != len(B):
            raise ValueError("El numero de columnas de la matriz A debe ser igual al numero de filas de la matriz B")

        result = [[0 for _ in range(len(B[0]))] for _ in range(len(A))]

        for i in range(len(A)):
            for j in range(len(B[0])):
                for k in range(len(B)):
                    result[i][j] += A[i][k] * B[k][j]
        return result
    
    def get_H(self,wr,wl,dt):
        vel=self.radius*(wr+wl)/2
        x= -vel*np.sin(self.theta)*dt
        y= vel*np.cos(self.theta)*dt 
        h=[[1, 1, x],
           [0, 0, y],
           [0, 0,  1 ]]
        return h

    def get_Qk(self,wr,wl,dt):
        const=self.radius*dt
        lin_wl=[[const*np.cos(self.theta)/2, const*np.cos(self.theta)/2],
                [const*np.sin(self.theta)/2, const*np.sin(self.theta)/2],
                [const/self.wheelbase, -const/self.wheelbase]]
        covariance_qk=[[wr, 0],
                        [0, wl ]]
    
        lin_wl_t=self.transpose_matrix(lin_wl)
        product=self.multiply_3x2_by_2x2(lin_wl,covariance_qk)
        Qk=self.multiply_3x2_by_2x2(product,lin_wl_t)
        return Qk 

    def get_covariance(self,H,Covariance,Qk):
        #A*B*AT+C
        A_B=self.multiply_matrices(H,Covariance)
        H_t=self.transpose_matrix(H)
        product=self.multiply_matrices(A_B,H_t)
        Covariance=self.add_matrices(product,Qk)
        return Covariance



    def calculate_odometry(self):
        current_time = rospy.Time.now()  # Get current time
    
        if self.first:
            self.previous_time = current_time
            self.first = False
        else:
            dt = (current_time - self.previous_time).to_sec()  # get dt
            self.previous_time = current_time

            #Matrix 
            #get the Qk
            self.Qk_matrix=self.get_Qk(self.wr_speed,self.wl_speed,dt)
            #get H of linearised model// Here to compute the pos 0
            self.H_matrix=self.get_H(self.wr_speed,self.wl_speed,dt)
            #Get the covariance 
            self.covariance_matrix=self.get_covariance(self.H_matrix,self.covariance_matrix,self.Qk_matrix)
            #get the pose of the robot
            
            
            self.get_positon(self.wr_speed,self.wl_speed,dt)

            # Create Odometry message
            odom_msg = self.get_odometry(current_time,self.x,self.y,self.theta,self.covariance_matrix)
            print(self.theta)
            
            # Publish Odometry message
            self.odom_pub.publish(odom_msg)
    
            # Publish transform
            self.transform(odom_msg)
    
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
