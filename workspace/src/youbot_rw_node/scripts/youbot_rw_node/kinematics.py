#!/usr/bin/env python
import PyKDL as kdl
import math
import rospy

from std_msgs.msg import Float64

ALMOST_PLUS_ONE=0.9999999
ALMOST_MINUS_ONE=-0.9999999

min_angles_ = [math.radians(-169.0),math.radians(-65.0),math.radians(-151.0),math.radians(-102.0),math.radians(-167.0)]
max_angles_ = [math.radians(169.0),math.radians(90.0),math.radians(146.0),math.radians(102.0),math.radians(167.0)]

print min_angles_
print max_angles_

def cleanFrame ( frame ):
    frame = kdl.Frame.Identity()
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
    l0x = 0.024
    l0z = 0.096
    l1x = 0.033
    l1z = 0.019
    l2 = 0.155
    l3 = 0.135

    # Distance from arm_link_4 to arm_link_5
    d = 0.13

    j1 = 0.0
    j2 = 0.0
    j3 = 0.0
    j4 = 0.0
    j5 = 0.0


    # Transform from frame 0 to frame 1
    frame0_to_frame1 = kdl.Frame(kdl.Rotation.Identity(),  kdl.Vector(-l0x, 0.0, -l0z))
    g1 = frame0_to_frame1 * g0

    # First joint
    j1 = math.atan2(g1.p.y(), g1.p.x())
    if offset_joint_1:
        if j1 < 0:
            j1 += math.pi
        else:
            j1 -= math.pi

    # Transform from frame 1 to frame 2
    frame1_to_frame2 = kdl.Frame(kdl.Rotation.RPY(0,0,-j1), kdl.Vector(-l1x, 0.0 , -l1z))
    g2 =frame1_to_frame2 * g1

    # Project the frame into the plane of the arm
    g2_proj = projectGoalOrientationIntoArmSubspace(g2)


    # Set all values in the frame that are close to zero to exactly zero
    g2_proj = cleanFrame(g2_proj)

    # Fifth joint, determines the roll of the gripper (= wrist angle)
    s1 = math.sin(j1)
    c1 = math.cos(j1)
    r11 = g1.M[0, 0]
    r12 = g1.M[0, 1]
    r21 = g1.M[1, 0]
    r22 = g1.M[1, 1]
    j5 = math.atan2(r21 * c1 - r11 * s1, r22 * c1 - r12 * s1)


    # The sum of joint angles two to four determines the overall "pitch" of the end effector
    r13 = g2_proj.M[0, 2]
    r33 = g2_proj.M[2, 2]
    j234 = math.atan2(r13, r33)

    p2 = g2_proj.p

    # In the arm's subplane, offset from the end-effector to the fourth joint
    p2.x(p2.x() - d * math.sin(j234))
    p2.z(p2.z() - d * math.cos(j234))


    # Check if the goal position can be reached at all
    if (l2 + l3) < math.sqrt((p2.x() * p2.x()) + (p2.z() * p2.z())):
        return kdl.JntArray(0)

    # Third joint
    l_sqr = (p2.x() * p2.x()) + (p2.z() * p2.z())
    l2_sqr = l2 * l2
    l3_sqr = l3 * l3
    j3_cos = (l_sqr - l2_sqr - l3_sqr) / (2.0 * l2 * l3)

    if j3_cos > ALMOST_PLUS_ONE:
        j3 = 0.0
    elif j3_cos < ALMOST_MINUS_ONE:
        j3 = math.pi
    else:
        j3 = math.atan2(math.sqrt(1.0 - (j3_cos * j3_cos)), j3_cos)

    if offset_joint_3:
        j3 = -j3

    # Second joint
    t1 = math.atan2(p2.z(), p2.x())
    t2 = math.atan2(l3 * math.sin(j3), l2 + l3 * math.cos(j3))
    j2 = math.pi - t1 - t2


    # Fourth joint, determines the pitch of the gripper
    j4 = j234 - j2 - j3


    # This IK assumes that the arm points upwards, so we need to consider the offsets to the real home position
    offset1 = math.radians( 169.0)
    offset2 = math.radians(  65.0)
    offset3 = math.radians(-146.0)
    offset4 = math.radians( 102.5)
    offset5 = math.radians( 167.5)

    solution = kdl.JntArray(5)
    solution[0] = (offset1 - j1)
    solution[1] = (j2 + offset2)
    solution[2] = (j3 + offset3)
    solution[3] = (j4 + offset4)
    solution[4] = (offset5 - j5)

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
val=0.5
while not rospy.is_shutdown():
    tmp=CartToJnt(kdl.Frame(  kdl.Rotation.Identity()  ,  kdl.Vector(0, 0, 0.5) ))
    pub.publish(-0.191986)
    pub1.publish(0.64001)
    pub2.publish(-1.55513)
    pub3.publish(1.29037)
    pub4.publish(-0.218166)
    #pub.publish(tmp[0][0])
    #pub1.publish(tmp[0][1])
    #pub2.publish(tmp[0][2])
    #pub3.publish(tmp[0][3])
    #pub4.publish(tmp[0][4])
    print tmp[0]
    if dir==1:
        if val>1:
            dir=0
        val=val+0.1
    else:
        if val<0:
            dir=1
        val=val-0.1

    #print val

    r.sleep()