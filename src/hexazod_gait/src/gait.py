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

        #front left leg
        # fixed hight of coxa (z offset)
        z_offset = 0
        #  how far out (coord)
        y = 9
        # how far forward (coord) 8.5
        x = 3
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
        # self.calculate_angles(x, y, z_offset, 1)

    def calculate_angles(self, x, y, z_offset, leg):
        """
        calculates gamma
        out = tha alpha beta gamma of one leg
        in = the desired coordinate (x,y,z) of a leg(int), nubered leg (int)
        """
        leg_values = {"leg_num": leg, "gamma": 0, "alpha": 0, "beta": 0}

        g = math.hypot(x, y)
        print(g)
        # calculates alpha
        L = math.hypot(z_offset, g)
        print(("L: %s" % L))
        inside_alpha = (self.femer ** 2 + L ** 2 - self.tibia ** 2 ) / (2*self.femer * L)
        acos = math.acos(inside_alpha) #radians
        alpha = acos -1.4
        # print(("alpha2: %s " % alpha2))
        print(("alpha2 raid: %s " % ((acos*180)/ math.pi)))
        print(("alpha2 raid: %s " % alpha))
        #calculates beta
        inside_beta = (self.tibia**2 + self.femer**2 -L**2 ) / (2 * self.femer *self.tibia)
        acos_beta = math.acos(inside_beta)  #radians
        beta = acos_beta -.8
        #(acos_beta*180)/ math.pi

        # print("beta: %s" % self.beta)
        print(("beta raid: %s " % ((acos_beta*180)/ math.pi)))
        print(("beta raid: %s " % beta))
        #calculates gamma
        gamma = math.atan2(x , y) #radians

        print("gamma: %s" % ((gamma*180)/ math.pi))

        leg_values["gamma 1"] = gamma
        leg_values["alpha 2"] = alpha
        leg_values["beta 3"] = beta
        print(leg_values)
        self.send_values(leg_values)


    def send_values(self, leg_values):
        print("in send values")
        position = [0.0]*18
        rate = rospy.Rate(50) # 10hz
        leg_num = leg_values["leg_num"]*3
        for x in range((leg_num-3), leg_num): #to save in right position
            for i in range(1,3):
                for keys in leg_values.keys():
                    str_i = str(i)
                    if str_i in keys:
                        print("found %s " % keys)
                        position[x] = leg_values[keys]

        for i in range(100):
            print("pub %i" %i)
            gait_state = JointState()
            gait_state.header = Header()
            gait_state.header.stamp = rospy.Time.now()
            gait_state.name = ["coxa_joint_r1", "femur_joint_r1", "tibia_joint_r1", "coxa_joint_r2", "femur_joint_r2", "tibia_joint_r2","coxa_joint_r3", "femur_joint_r3", "tibia_joint_r3", "coxa_joint_l1", "femur_joint_l1", "tibia_joint_l1","coxa_joint_l2", "femur_joint_l2", "tibia_joint_l2", "coxa_joint_l3", "femur_joint_l3", "tibia_joint_l3"]

            gait_state.position = position
            #[gamma, alpha,beta, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            gait_state.velocity = []
            gait_state.effort = []
            self.pub.publish(gait_state)
            rate.sleep()

def main():
    #front left leg
    # fixed hight of coxa (z offset)
    z_offset = 0
    #  how far out (coord)
    y = 9
    # how far forward (coord) 8.5
    x = 6
    #zero out
    this = HexazodGait()
    this.calculate_angles(x, y, z_offset, 1)
    rospy.spin()
if __name__ == '__main__':
    try:
	    main()
    except rospy.ROSInterruptException:
        pass
