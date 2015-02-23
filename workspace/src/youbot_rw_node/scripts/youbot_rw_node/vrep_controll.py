__author__ = 'storm'

import rospy
from vrep_common.srv import *


def pauseSimualtion():
    """ pause Simulation in VRep
    """
    rospy.wait_for_service('/vrep/simRosPauseSimulation')
    try:
        ret = rospy.ServiceProxy('/vrep/simRosPauseSimulation', simRosPauseSimulation)
        print "Simulation paused"
        return ret.call()
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
        return -1


def startSimualtion():
    """ start Simulation in VRep
    """
    rospy.wait_for_service('/vrep/simRosStartSimulation')
    try:
        ret = rospy.ServiceProxy('/vrep/simRosStartSimulation', simRosStartSimulation)
        print "Simulation started"
        return ret.call()
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
        return -1


def stopSimualtion():
    """ stop Simulation in VRep
    """
    rospy.wait_for_service('/vrep/simRosStopSimulation')
    try:
        ret = rospy.ServiceProxy('/vrep/simRosStopSimulation', simRosStopSimulation)
        print "Simulation stopped"
        return ret.call()
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
        return -1


def setSyncSimualtion():
    """ set Simulation in VRep to trigger mode
    """
    rospy.wait_for_service('/vrep/simRosSynchronous')
    try:
        ret = rospy.ServiceProxy('/vrep/simRosSynchronous', simRosSynchronous)
        print "Set simulation into trigger mode"
        return ret.call(True)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
        return -1

def unsetSyncSimualtion():
    """ set Simulation in VRep to non trigger mode
    """
    rospy.wait_for_service('/vrep/simRosSynchronous')
    try:
        ret = rospy.ServiceProxy('/vrep/simRosSynchronous', simRosSynchronous)
        print "Set simulation into non trigger mode"
        return ret.call(False)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
        return -1


def TriggerSimualtion():
    """ triggers an simulation step in VRep
    """
    # rospy.wait_for_service('/vrep/simRosSynchronousTrigger')
    try:
        ret = rospy.ServiceProxy('/vrep/simRosSynchronousTrigger', simRosSynchronousTrigger)
        return ret.call()
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
        return -1