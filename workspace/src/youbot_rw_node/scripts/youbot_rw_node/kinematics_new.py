#!/usr/bin/env python
from cmath import atan
import math
import numpy as np
from numpy import sin, cos, arctan2, arccos
from numpy.linalg import norm

import PyKDL as kdl
from numpy.core.umath import arccos
import rospy
from std_msgs.msg import Float64


ALMOST_PLUS_ONE=0.9999999
ALMOST_MINUS_ONE=-0.9999999

min_angles_ = [math.radians(-169.0),math.radians(-65.0),math.radians(-151.0),math.radians(-102.0),math.radians(-167.0)]
max_angles_ = [math.radians(169.0),math.radians(90.0),math.radians(146.0),math.radians(102.0),math.radians(167.0)]


dh=list()
dh.append({'theta':0           ,'d':-0.05    ,'a':-0.23   ,'alpha':0               })  #from write plane to joint_1 (KS0)
dh.append({'theta':0           ,'d':0.147   ,'a':0.033    ,'alpha':math.pi/2       })  #from KS0 to joint 2 (KS1)
dh.append({'theta':math.pi/2   ,'d':0       ,'a':0.155    ,'alpha':0               })  #from KS1 to joint 3 (KS2)
dh.append({'theta':0           ,'d':0       ,'a':0.135    ,'alpha':0               })  #from KS2 to joint 4 (KS3)
dh.append({'theta':math.pi/2   ,'d':0       ,'a':0        ,'alpha':math.pi/2       })  #from KS3 to joint 5 (KS4)
dh.append({'theta':0           ,'d':0.2175  ,'a':0        ,'alpha':0               })  #from KS4 to tcp
dh.append({'theta':0           ,'d':0.03    ,'a':0        ,'alpha':0               })  #from tcp to pencil


def cleanFrame ( frame ):
    #frame = kdl.Frame.Identity()
    for i in range(0,3):
        for j in range(0,3):
            if math.fabs(frame.M[i,j]) < 0.000001:
                frame.M[i,j]=0
    return frame

def get_dh_transform(dh, theta=0):
    trans = np.matrix((    (cos(dh['theta']+theta),    -sin(dh['theta']+theta)*cos(dh['alpha']),       sin(dh['theta']+theta)*sin(dh['alpha']),       dh['a'] *cos(dh['theta']+theta)),
        (sin(dh['theta']+theta),                      cos(dh['theta']+theta)*cos(dh['alpha']),       -cos(dh['theta']+theta)*sin(dh['alpha']),       dh['a']*sin(dh['theta']+theta)),
        (0,                                     sin(dh['alpha']),                         cos(dh['alpha']),                        dh['d']),
        (0,                                     0,                                        0,                                       1)))
    return trans


def get_dh_transform(dh, theta=0):
    trans = np.matrix((    (cos(dh['theta']+theta),    -sin(dh['theta']+theta)*cos(dh['alpha']),       sin(dh['theta']+theta)*sin(dh['alpha']),       dh['a'] *cos(dh['theta']+theta)),
        (sin(dh['theta']+theta),                      cos(dh['theta']+theta)*cos(dh['alpha']),       -cos(dh['theta']+theta)*sin(dh['alpha']),       dh['a']*sin(dh['theta']+theta)),
        (0,                                     sin(dh['alpha']),                         cos(dh['alpha']),                        dh['d']),
        (0,                                     0,                                        0,                                       1)))
    return trans


def get_inv_transform(dh, theta=0):
    trans = np.matrix((  (cos(dh['theta']+theta),       sin(dh['theta']+theta),                         0,                                              -dh['a']),
        (-sin(dh['theta']+theta)*cos(dh['alpha']),      cos(dh['theta']+theta)*cos(dh['alpha']),        sin(dh['alpha']),                               -dh['d']*sin(dh['alpha'])),
        (sin(dh['alpha'])*sin(dh['theta']+theta),       -cos(dh['theta']+theta)*sin(dh['alpha']),       cos(dh['alpha']),                               -dh['d']*cos(dh['alpha'])),
        (0,                                             0,                                              0,                                              1)))
    return trans

def get_transformation2wrist_point(alpha):
    return np.matrix((  (cos(alpha),0,sin(alpha), 0),
                (0  ,   1   ,   0, 0),
                (-sin(alpha),   0,  cos(alpha), dh[5]['d']+dh[6]['d']),
                (0, 0, 0, 1)
    ))

def inverse_kin( point ):
    """ calculate the inverse kinematics

    :param point on write plane
    :type trajectory_angles: array of floats
    :returns: possible solutions of inverse kinematic
    :rtype: list of( arrays of floats)
    """

    print 'point', point
    #point[2]=point[2]+dh[5]['d']+dh[6]['d']             # offset calculate Wrist point under condition that Wrist is vertical up on write plane
    wp= np.matrix(np.resize(point,4)).transpose()
    wp[3]=1
    wp = get_transformation2wrist_point(45.0/180*np.pi) * wp
    print 'wpoint',wp
    wp_0=get_inv_transform(dh[0])*wp                    # transform wp into KS0
    theta_0 = np.empty([2])
    theta_0[0]=arctan2(wp_0[1],wp_0[0])                 # turn robot arm into wrist point plane

    if theta_0[0] < 0:
        theta_0[1]=theta_0[0]+np.pi
    else:
        theta_0[1]=theta_0[0]-np.pi
    wp_1 = np.array([get_inv_transform(dh[1],theta_0[0])*wp_0, get_inv_transform(dh[1],theta_0[1])*wp_0])       # numpy array of 2 points

    print theta_0
    ##d_wp_1 = np.array((norm(wp_1[0]),norm(wp_1[1])))
    ##print d_wp_1
    d_wp_1 = np.array(( np.sqrt((wp_1[0][0]**2 + wp_1[0][1]**2)),np.sqrt((wp_1[1][0]**2 + wp_1[1][1]**2)) ))    # array of 2 distances
    print d_wp_1

    s=np.array([arctan2(wp_1[0][1],wp_1[0][0]),arctan2(wp_1[1][1],wp_1[1][0])])                                 # array of 2 angles
    print  (dh[3]['a']**2-d_wp_1[0]**2-dh[2]['a']**2) / (-2*d_wp_1[0]*dh[2]['a'])
    print  (dh[3]['a']**2-d_wp_1[1]**2-dh[2]['a']**2) / (-2*d_wp_1[1]*dh[2]['a'])
    r=np.array([arccos( (dh[3]['a']**2-d_wp_1[0]**2-dh[2]['a']**2) / (-2*d_wp_1[0]*dh[2]['a']) ),
                arccos( (dh[3]['a']**2-d_wp_1[1]**2-dh[2]['a']**2) / (-2*d_wp_1[1]*dh[2]['a']) )])              # array of 2 angles

    theta_1_0 = np.empty([2])                                                                                   #array of 2 angles
    theta_1_1 = np.empty([2])                                                                                   #array of 2 angles

    theta_1_0[0] =s[0]+r[0]
    theta_1_0[1] =s[0]-r[0]
    theta_1_1[0] =s[1]+r[1]
    theta_1_1[1] =s[1]-r[1]

    wp_2=  np.array([get_inv_transform(dh[2],theta_1_0[0])*wp_1[0], get_inv_transform(dh[2],theta_1_0[1])*wp_1[0] ,     # numpy array of 4 points
                     get_inv_transform(dh[2],theta_1_1[0])*wp_1[1], get_inv_transform(dh[2],theta_1_1[1])*wp_1[1]])

    theta_2_0 = np.empty([2])                                                                                   #array of 2 angles
    theta_2_1 = np.empty([2])                                                                                   #array of 2 angles

    theta_2_0[0] = arctan2(wp_2[0][1], wp_2[0][0])
    theta_2_0[1] = arctan2(wp_2[1][1], wp_2[1][0])
    theta_2_1[0] = arctan2(wp_2[2][1], wp_2[2][0])
    theta_2_1[1] = arctan2(wp_2[3][1], wp_2[3][0])


    wp_3=  np.array([get_inv_transform(dh[3],theta_2_0[0])*wp_2[0], get_inv_transform(dh[3],theta_2_0[1])*wp_2[0] ,     # numpy array of 4 points
                     get_inv_transform(dh[3],theta_2_1[0])*wp_2[1], get_inv_transform(dh[3],theta_2_1[1])*wp_2[1]])

    theta_3_0 = np.empty([2])                                                                                   #array of 2 angles
    theta_3_1 = np.empty([2])                                                                                   #array of 2 angles

    theta_3_0[0] = arctan2(wp_3[0][1], wp_3[0][0])
    theta_3_0[1] = arctan2(wp_3[1][1], wp_3[1][0])
    theta_3_1[0] = arctan2(wp_3[2][1], wp_3[2][0])
    theta_3_1[1] = arctan2(wp_3[3][1], wp_3[3][0])



    result = list()
    result.append(np.array([ theta_0[0], theta_1_0[0], theta_2_0[0], theta_3_0[0], 0.0 ]))
    result.append(np.array([ theta_0[0], theta_1_0[1], theta_2_0[1], theta_3_0[1], 0.0 ]))
    result.append(np.array([ theta_0[1], theta_1_1[0], theta_2_1[0], theta_3_1[0], 0.0 ]))
    result.append(np.array([ theta_0[1], theta_1_1[1], theta_2_1[1], theta_3_1[1], 0.0 ]))

    for i in result:
        print i/np.pi*180.
        print isSolutionValid(i)
    return result


def CartToJnt(p_in):
    solutions = list()

    bools = [ True , False ]
    # iterate over all redundant solutions
    for i in range(0,2):
        for j in range(0,2):
            solution = inverse_kin(p_in, bools[i], bools[j])
            if isSolutionValid(solution):
                solutions.append(solution)

    return solutions


def isSolutionValid(solution):
    if len(solution) != 5:
        return False
    for i in range(0,len(solution)):
        if solution[i] < min_angles_[i] or solution[i] > max_angles_[i]:
            return False



    return True


rospy.init_node('kinPublisher', anonymous=True)


def offset2world(thetas):
    """ todo

    :param todo
    :type todo
    :returns: todo
    :rtype: todo
    """

    return -thetas


def direct_kin(thetas):
    trans=get_dh_transform(dh[0]) * get_dh_transform(dh[1],thetas[0]) * get_dh_transform(dh[2],thetas[1]) * \
          get_dh_transform(dh[3],thetas[2]) * get_dh_transform(dh[4],thetas[3]) * get_dh_transform(dh[5],thetas[4])
    return (trans*np.matrix((0,0,0,1)).transpose())[:3]



def kin():
    #print direct_kin([0,0,0,0,0])
    inverse_kin(np.matrix((0.1,  0,  0.0)).transpose())



pub = rospy.Publisher('/youbot_rw/vrep/arm_joint1_target', Float64, queue_size=10)
pub1 = rospy.Publisher('/youbot_rw/vrep/arm_joint2_target', Float64, queue_size=10)
pub2 = rospy.Publisher('/youbot_rw/vrep/arm_joint3_target', Float64, queue_size=10)
pub3 = rospy.Publisher('/youbot_rw/vrep/arm_joint4_target', Float64, queue_size=10)
pub4 = rospy.Publisher('/youbot_rw/vrep/arm_joint5_target', Float64, queue_size=10)

r = rospy.Rate(10) # 10hz
dir=1
val=0.0
kin()
#
# while not rospy.is_shutdown():
#     #tmp=CartToJnt(kdl.Frame(  kdl.Rotation.Identity()  ,  kdl.Vector(0.0, 0.1, 0.0) ))
#     #print tmp
#
#
#     pub.publish(0)
#     pub1.publish(0)
#     pub2.publish(0)
#     pub3.publish(0)
#     pub4.publish(0)
#     #print tmp
#     #if dir==1:
#     #    if val>0.4:
#     #        dir=0
#     #    val=val+0.01
#     #else:
#     #    if val<-0.4:
#     #        dir=1
#     #    val=val-0.01
#
#     #print val
#
#     r.sleep()