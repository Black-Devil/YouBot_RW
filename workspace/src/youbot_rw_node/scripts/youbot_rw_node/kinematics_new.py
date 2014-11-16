#!/usr/bin/env python
import PyKDL as kdl
import math
import rospy
import numpy as np

from std_msgs.msg import Float64

ALMOST_PLUS_ONE=0.9999999
ALMOST_MINUS_ONE=-0.9999999

min_angles_ = [math.radians(-169.0),math.radians(-65.0),math.radians(-151.0),math.radians(-102.0),math.radians(-167.0)]
max_angles_ = [math.radians(169.0),math.radians(90.0),math.radians(146.0),math.radians(102.0),math.radians(167.0)]


def cleanFrame ( frame ):
    #frame = kdl.Frame.Identity()
    for i in range(0,3):
        for j in range(0,3):
            if math.fabs(frame.M[i,j]) < 0.000001:
                frame.M[i,j]=0
    return frame


def projectGoalOrientationIntoArmSubspace( goal ):
    """
    :param goal: kdl.Frame
    :rtype : kdl.Frame
    """
    y_t_hat = goal.M.UnitY()
    z_t_hat = goal.M.UnitZ()

    # m_hat is the normal of the "arm plane"
    m_hat = kdl.Vector(0, -1, 0)

    # k_hat is the vector about which rotation of the goal frame is performed
    k_hat = m_hat * z_t_hat         # cross product

    # z_t_hat_tick is the new pointing direction of the arm
    z_t_hat_tick = k_hat * m_hat     # cross product

    # the amount of rotation
    cos_theta = kdl.dot(z_t_hat , z_t_hat_tick)

    # first cross product then dot product
    sin_theta = kdl.dot(z_t_hat * z_t_hat_tick, k_hat)

    # use Rodrigues' rotation formula to perform the rotation
    # k_hat * y_t_hat is cross product
    y_t_hat_tick = (cos_theta * y_t_hat) + (sin_theta * (k_hat * y_t_hat)) + (1 - cos_theta) * (kdl.dot(k_hat, y_t_hat)) * k_hat

    x_t_hat_tick = y_t_hat_tick * z_t_hat_tick  # cross product

    rot = kdl.Rotation( x_t_hat_tick, y_t_hat_tick, z_t_hat_tick )

    # the frame uses the old position but has the new, projected orientation
    return kdl.Frame(rot, goal.p)



def inverseKinematics(  g0  , offset_joint_1  , offset_joint_3  ):
    # Parameters from youBot URDF file
    dh=list()
    dh.append({'theta':0           ,'d':0.07    ,'a':0.23     ,'alpha':0               })  #to write plane
    dh.append({'theta':0           ,'d':0.147   ,'a':0.033    ,'alpha':math.pi/2       })  #to joint 2
    dh.append({'theta':0           ,'d':0       ,'a':0.155    ,'alpha':0               })  #to joint 3
    dh.append({'theta':0           ,'d':0       ,'a':0.135    ,'alpha':0               })  #to joint 4
    dh.append({'theta':math.pi/2   ,'d':0       ,'a':0        ,'alpha':math.pi/2       })  #to joint 5
    dh.append({'theta':0           ,'d':0.2175  ,'a':0        ,'alpha':0               })  #to tcp
    dh.append({'theta':0           ,'d':0.03    ,'a':0        ,'alpha':0               })  #to pencil

    tmp=kdl.Frame().DH(dh[6]['a'],dh[6]['alpha'],dh[6]['d'],dh[6]['theta'])
    tmp2=kdl.Frame().DH(dh[5]['a'],dh[5]['alpha'],dh[5]['d'],dh[5]['theta'])

    g0_off=tmp2*tmp*g0

    print "G0: ",g0.p
    print "G0_off: ",g0_off.p


    tmp = kdl.Frame().DH(dh[0]['a'],dh[0]['alpha'],dh[0]['d'],dh[0]['theta'])

    g1=tmp*g0_off

    print "G01: ",g1.p

    j1 = math.atan2(g1.p.y(), g1.p.x())
    j4 = j1
    if offset_joint_1:
        if j1 < 0:
            j1 += math.pi
        else:
            j1 -= math.pi

    print "J1: ",j1

    print "d[1]",j1+dh[1]['theta']

    tmp = kdl.Frame().DH(dh[1]['a'], dh[1]['alpha'], dh[1]['d'], j4+dh[1]['theta'])
    frame1_to_frame2 = kdl.Frame(kdl.Rotation.RPY(dh[1]['alpha'],0,j4), kdl.Vector(dh[1]['a'], 0.0 , dh[1]['d']))
    g2=frame1_to_frame2*g1

    print "G2_dh: ",(tmp*g1).p
    print "G2_ro: ",g2.p

    vec_25=math.sqrt(g2.p[0]**2 + g2.p[0]**2 + g2.p[0]**2)
    print "VEC_25: ",vec_25


    s_angle = math.atan2(g2.p.y(),g2.p.x())
    print s_angle

    w=0.135
    a=0.155

    r_angle=math.acos((w**2-vec_25**2-a**2/2*vec_25-a))

    if offset_joint_3:
        j2=s_angle-r_angle
    else:
        j2=s_angle+r_angle











    # This IK assumes that the arm points upwards, so we need to consider the offsets to the real home position
    offset1 = math.radians( 169.0)
    offset2 = math.radians(  65.0)
    offset3 = math.radians(-146.0)
    offset4 = math.radians( 102.5)
    offset5 = math.radians( 167.5)

    solution = kdl.JntArray(5)
    solution[0] = j1#(offset1 - j1)
    solution[1] = j2#(j2 + offset2)
    solution[2] = 0#(j3 + offset3)
    solution[3] = 0#(j4 + offset4)
    solution[4] = 0#(offset5 - j5)

    print "J2: ",j2

    for i in range(0,5):
        while solution[i]>math.pi:
            solution[i]=solution[i]-(math.pi*2)
        while solution[i]<-math.pi:
            solution[i]=solution[i]+(math.pi*2)



    return solution


def CartToJnt(p_in):
    solutions = list()

    bools = [ True , False ]
    # iterate over all redundant solutions
    for i in range(0,2):
        for j in range(0,2):
            solution = inverseKinematics(p_in, bools[i], bools[j])
            if isSolutionValid(solution):
                solutions.append(solution)

    return solutions


def isSolutionValid(solution):
    if solution.rows() != 5:
        return False
    for i in range(0,solution.rows()):
        if solution[i] < min_angles_[i] or solution[i] > max_angles_[i]:
            return False

    return True


rospy.init_node('kinPublisher', anonymous=True)


pub = rospy.Publisher('/youbot_rw/vrep/arm_joint1_target', Float64, queue_size=10)
pub1 = rospy.Publisher('/youbot_rw/vrep/arm_joint2_target', Float64, queue_size=10)
pub2 = rospy.Publisher('/youbot_rw/vrep/arm_joint3_target', Float64, queue_size=10)
pub3 = rospy.Publisher('/youbot_rw/vrep/arm_joint4_target', Float64, queue_size=10)
pub4 = rospy.Publisher('/youbot_rw/vrep/arm_joint5_target', Float64, queue_size=10)

r = rospy.Rate(10) # 10hz
dir=1
val=0.0
while not rospy.is_shutdown():
    tmp=CartToJnt(kdl.Frame(  kdl.Rotation.Identity()  ,  kdl.Vector(0.0, 0.0, 0.0) ))
    print tmp
    pub.publish(tmp[0][0])
    pub1.publish(tmp[0][1])
    pub2.publish(tmp[0][2])
    pub3.publish(tmp[0][3])
    pub4.publish(tmp[0][4])
    #print tmp
    if dir==1:
        if val>0.4:
            dir=0
        val=val+0.01
    else:
        if val<-0.4:
            dir=1
        val=val-0.01

    #print val

    r.sleep()