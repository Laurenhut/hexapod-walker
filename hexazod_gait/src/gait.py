#!/usr/bin/env python
import rospy
import sys
import numpy
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


class HexazodGait(object):

    def __init__(self):
        rospy.init_node('joint_states', anonymous=True)
        self.pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        # fixed hight of coxa (z offset)
        self.z_offset = 0
        #  how far out (coord)
        self.y = 9
        # how far forward (coord) 8.5
        self.x = 3
        #zero out
        """
        self.z_offset = 0
        # how far forward (coord) 8.5
        self.x = 3
        #  how far out (coord)
        self.y = 9
        """

        """
            raised
            self.z_offset = 1
            # how far forward (coord) 8.5
            self.x = -3
            #  how far out (coord)
            self.y = -2
        """
        self.femer = 9.5 # 2 alpha
        self.tibia = 12.5 # 3 beta - r in spherical
        self.coxa = 3.75  # 1 gamma

    def calculate_angles(self):
        """
        calculates gamma
        """
        g = math.hypot(self.x,self.y)
        print(g)
        # calculates alpha
        self.L = math.hypot(self.z_offset,g)
        print(("L: %s" % self.L))
        inside_alpha = (self.femer ** 2 + self.L ** 2 - self.tibia ** 2 ) / (2*self.femer * self.L)
        acos = math.acos(inside_alpha)
        self.alpha = acos -1.4
        # print(("alpha2: %s " % alpha2))
        print(("alpha2 raid: %s " % ((acos*180)/ math.pi)))
        print(("alpha2 raid: %s " % self.alpha))
        #calculates beta
        inside_beta = (self.tibia**2 + self.femer**2 -self.L**2 ) / (2 * self.femer *self.tibia)
        acos_beta = math.acos(inside_beta)
        self.beta = acos_beta -.8
        #(acos_beta*180)/ math.pi

        # print("beta: %s" % self.beta)
        print(("beta raid: %s " % ((acos_beta*180)/ math.pi)))
        print(("beta raid: %s " % self.beta))
        #calculates gamma
        self.gamma = math.atan(self.x / self.y)

        print("gamma: %s" % ((self.gamma*180)/ math.pi))
        self.send_values()


    def send_values(self):
        print("hi")
        rate = rospy.Rate(50) # 10hz
        for i in range(100):
            gait_state = JointState()
            gait_state.header = Header()
            gait_state.header.stamp = rospy.Time.now()
            gait_state.name = ["coxa_joint_r1", "femur_joint_r1", "tibia_joint_r1", "coxa_joint_r2", "femur_joint_r2", "tibia_joint_r2","coxa_joint_r3", "femur_joint_r3", "tibia_joint_r3", "coxa_joint_l1", "femur_joint_l1", "tibia_joint_l1","coxa_joint_l2", "femur_joint_l2", "tibia_joint_l2", "coxa_joint_l3", "femur_joint_l3", "tibia_joint_l3"]

            gait_state.position = [self.gamma, self.alpha,self.beta, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            gait_state.velocity = []
            gait_state.effort = []
            self.pub.publish(gait_state)
            rate.sleep()

def main():
    this = HexazodGait()
    this.calculate_angles()
    rospy.spin()
if __name__ == '__main__':
    try:
	    main()
    except rospy.ROSInterruptException:
        pass
