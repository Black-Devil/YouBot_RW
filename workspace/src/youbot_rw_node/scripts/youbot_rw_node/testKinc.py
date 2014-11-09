#!/usr/bin/env python
import KinematicFunctions as kinFunc
import rospy
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState


def callback(data):
    #print str(data.position[10])+"   "+str(data.position[10])+"   "+str(data.position[11])+"   "+str(data.position[12])+"   "+str(data.position[13])+"   "+str(data.position[14])
    i=0

def test():
    pub = rospy.Publisher('/youbot_rw/vrep/arm_joint1_target', Float64, queue_size=10)
    pub1 = rospy.Publisher('/youbot_rw/vrep/arm_joint2_target', Float64, queue_size=10)
    pub2 = rospy.Publisher('/youbot_rw/vrep/arm_joint3_target', Float64, queue_size=10)
    pub3 = rospy.Publisher('/youbot_rw/vrep/arm_joint4_target', Float64, queue_size=10)
    pub4 = rospy.Publisher('/youbot_rw/vrep/arm_joint5_target', Float64, queue_size=10)

    rospy.Subscriber("/vrep/youbot_rw/joint_states", JointState, callback)
    rospy.init_node('kinPublisher', anonymous=True)
    r = rospy.Rate(100) # 10hz
    i=0

    sol = kinFunc.CalculateInverseKinematics(np.array([[0,0,0.4,0],[0,0,0.4,0],[0,0,0.4,0],[0,0,0,1]]))
    for i in sol:
        print kinFunc.CheckAngleConstraints(i)


    while not rospy.is_shutdown():
        pub.publish(np.deg2rad(0))
        pub1.publish(np.deg2rad(0))
        pub2.publish(np.deg2rad(0))
        pub3.publish(np.deg2rad(0))
        pub4.publish(np.deg2rad(0))


        r.sleep()

if __name__ == '__main__':

    try:
        test()
    except rospy.ROSInterruptException: pass