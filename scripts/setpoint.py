#!/usr/bin/env python
import rospy

class PointGen:
    def __init__(self):
        self.x = []
        self.y = []
        self.index = int

class PointGenNode:
    def __init__(self):
        self.cmd_pub = rospy.Publisher('/set_point', PointGen, queue_size=10)  
        
        self.PointGen = PointGen()
        self.PointGen.x = [2, 2, 0, 0]
        self.PointGen.y = [0, 2, 2, 0]
        self.PointGen.index = len(self.PointGen.x)

if __name__ == '__main__':

    rospy.init_node("PointGenNode")
    SPN = PointGenNode()
    SPN.cmd_pub.publish(SPN.PointGen)

    rospy.spin()