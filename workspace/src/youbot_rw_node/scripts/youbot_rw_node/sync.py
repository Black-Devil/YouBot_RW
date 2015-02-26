__author__ = 'storm'

import numpy as np
import rospy
from vrep_common.srv import *
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from vrep_controll import *
joints = [0, 0, 0, 0, 0]
err_old = np.array(joints)

position_geted=False


def joint_callback(data):
    """ Joint callback, sets the Joint positions
    """
    joints[0] = data.position[9]
    joints[1] = data.position[10]
    joints[2] = data.position[11]
    joints[3] = data.position[12]
    joints[4] = data.position[13]
    global position_geted
    position_geted = True



def getJointPostition():
    """ get Joint States
    @retval <Array> with Joint positions in rad
    """
    global position_geted
    position_geted=False
    while not position_geted:
        TriggerSimualtion()
    return joints


def init_sync():
    """ Initialize all needet Ros-Subscribers
    """
    rospy.Subscriber("/vrep/youbot_rw/joint_states", JointState, joint_callback)



def wait_untel_pos(target_pos):
    """ wait until position is reached, wait for error threshold
    @param target_pos Postion to reach
    """
    mini = 0.003
    err0 = abs(target_pos[0] - joints[0])
    err1 = abs(target_pos[1] - joints[1])
    err2 = abs(target_pos[2] - joints[2])
    err3 = abs(target_pos[3] - joints[3])
    err4 = abs(target_pos[4] - joints[4])
    while err0 > mini or err1 > mini or err2 > mini or err3 > mini or err4 > mini:
        TriggerSimualtion()
        err0 = abs(target_pos[0] - joints[0])
        err1 = abs(target_pos[1] - joints[1])
        err2 = abs(target_pos[2] - joints[2])
        err3 = abs(target_pos[3] - joints[3])
        err4 = abs(target_pos[4] - joints[4])


def wait_untel_pos_eq(target_pos):
    """ wait until position is reached, wait for stable error
    @param target_pos Postion to reach
    """
    global joints
    TriggerSimualtion()
    err = abs(np.array(target_pos) - np.array(joints))
    global err_old
    global position_geted
    while (err != err_old).all() or not position_geted:
        global err_old
        global position_geted
        global joints
        err_old = err
        TriggerSimualtion()
        #sleep(0.1)
    position_geted=False