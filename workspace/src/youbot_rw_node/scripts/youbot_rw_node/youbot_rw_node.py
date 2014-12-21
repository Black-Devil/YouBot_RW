#!/usr/bin/env python

from time import sleep, time
import math
import threading

import rospy
from std_msgs.msg import *
from sensor_msgs.msg import *

import numpy as np

import kinematics_geom as kin


import status_intf as status
from youbot_rw_rqt_gui.msg import *


class Node(object):
    def __init__(self):
        super(Node, self).__init__()
        self.run = True
        self.pause = False

        self.lock = threading.Lock()

        rospy.init_node('youbot_rw_node')
	
        self.init_params()

        #self.pub_yaw_cmd = rospy.Publisher('/youbot_rw/node_output', Float32, tcp_nodelay=True,queue_size=1)
        
        self.run_status = True
        self.spin_count = 0
        self.r = rospy.Rate(self.rate)
        
        rospy.Subscriber('/youbot_rw/gui2node', rw_node, self.callback_write_cmd)
        self.pub2gui = rospy.Publisher('/youbot_rw/node2gui', rw_node_state, tcp_nodelay=True,queue_size=1)
        # Publisher for vrep interface
        self.pub2vrep_joint_1_trgt = rospy.Publisher('/youbot_rw/vrep/arm_joint1_target', Float64, tcp_nodelay=True,queue_size=1)
        self.pub2vrep_joint_2_trgt = rospy.Publisher('/youbot_rw/vrep/arm_joint2_target', Float64, tcp_nodelay=True,queue_size=1)
        self.pub2vrep_joint_3_trgt = rospy.Publisher('/youbot_rw/vrep/arm_joint3_target', Float64, tcp_nodelay=True,queue_size=1)
        self.pub2vrep_joint_4_trgt = rospy.Publisher('/youbot_rw/vrep/arm_joint4_target', Float64, tcp_nodelay=True,queue_size=1)
        self.pub2vrep_joint_5_trgt = rospy.Publisher('/youbot_rw/vrep/arm_joint5_target', Float64, tcp_nodelay=True,queue_size=1)
        
        rospy.Subscriber('/vrep/youbot_rw/joint_states', JointState, self.callback_vrep_joint_states)
        
        
        self.spin_restarter()        


    def init_params(self):
        self.rate = 100
        self.disable = False
        self.status = 1 #0= error 1= no error
        self.status_string = "no error"
        self.status_vrep = 0 #0=doing nothing 1= in progress 2= movement done
        self.kinematics = kin.Kinematics_geom()
                
        #do init here


    def spin_restarter(self):
        #print "waiting for button"
        while(not rospy.is_shutdown() and self.run):
            if(self.run_status):
                self.init_params()
                self.spin()                
            self.r.sleep()


    def spin(self):
        self.spin_count += 1
        if(self.spin_count>1):
            return

        rospy.loginfo("started")
        self.send_status2gui(status.STATUS_NODE_NO_ERROR, status.STATUS_VREP_WAITING_4_CMD, "controller ready for input")
        while(not rospy.is_shutdown() and self.run and self.run_status):            
            self.r.sleep()

        rospy.loginfo("stopped")
        self.spin_count -= 1


    def send_vrep_joint_targets(self, trgts, bogen):
        rospy.loginfo("send vrep joint targets")
        if bogen:
            trgts_bogen = np.array([(trgts[0]),
                                (trgts[1]),
                                (trgts[2]),
                                (trgts[3]),
                                (trgts[4])])
        else:
            trgts_bogen = np.array([(((2.*np.pi)/360.)*trgts[0]),
                                    (((2.*np.pi)/360.)*trgts[1]),
                                    (((2.*np.pi)/360.)*trgts[2]),
                                    (((2.*np.pi)/360.)*trgts[3]),
                                    (((2.*np.pi)/360.)*trgts[4])])

        self.pub2vrep_joint_1_trgt.publish(trgts_bogen[0])
        self.pub2vrep_joint_2_trgt.publish(trgts_bogen[1])
        self.pub2vrep_joint_3_trgt.publish(trgts_bogen[2])
        self.pub2vrep_joint_4_trgt.publish(trgts_bogen[3])
        self.pub2vrep_joint_5_trgt.publish(trgts_bogen[4])
    
    
     
    def callback_write_cmd(self,msg):
        if(not self.run_status):
            return

        self.lock.acquire()

        rospy.loginfo("write_cmd callback")        
        self.time = rospy.get_time()
        self.parse_input_from_gui(msg)
        self.send_status2gui( status.STATUS_NODE_NO_ERROR, status.STATUS_VREP_WAITING_4_CMD, "received write command")

	    #DO WRITING WITH ROBOT HERE
        #print self.config_use_thetas
        if(self.config_use_thetas == 1 and self.config_use_pos == 0):
            tmp = self.kinematics.offset2world(self.config_thetas)
            # TODO: check angle constrains
            self.send_vrep_joint_targets(tmp, False)
            self.config_pos = self.kinematics.direct_kin(np.deg2rad(tmp))
            print("pos: [%.4f; %.4f; %.4f]" % (self.config_pos[0],self.config_pos[1],self.config_pos[2]) )
            pos_wp = self.kinematics.direct_kin_2_wristPoint(np.deg2rad(tmp))
            print("pos_wp: [%.4f; %.4f; %.4f]" % (pos_wp[0],pos_wp[1],pos_wp[2]) )
            # todo: implement communication to gui

        if(self.config_use_pos == 1 and self.config_use_thetas == 0):
            #self.kinematics.debugTheta_1(wp_1 = np.matrix((0.145,  0.145,  0.0, 1.0)).transpose())
            #self.kinematics.debugTheta_1(wp_1 = np.matrix((-0.145,  0.145,  0.0, 1.0)).transpose())
            #self.kinematics.debugTheta_1(wp_1 = np.matrix((-0.145,   0.048,  0.0, 1.0)).transpose())
            #self.kinematics.debugTheta_1(wp_1 = np.matrix((-0.145,  -0.048,  0.0, 1.0)).transpose())
            tmpPos = np.matrix((self.config_pos[0],  self.config_pos[1],  self.config_pos[2])).transpose()
            #print "input ik", tmpPos
            valid_ik_solutions = self.kinematics.get_valid_inverse_kin_solutions(tmpPos)
            if not valid_ik_solutions:
                print "// no valid ik solution possible! //"
            else:
                print "// valid ik solution possible! //"
                #print "first valid ik solution: [%.4f; %.4f; %.4f; %.4f; %.4f;]" % (math.degrees(valid_ik_solutions[0][0]), math.degrees(valid_ik_solutions[0][1]), math.degrees(valid_ik_solutions[0][2]), math.degrees(valid_ik_solutions[0][3]), math.degrees(valid_ik_solutions[0][4]) ) , self.kinematics.isSolutionValid(valid_ik_solutions[0])
                #dk_pos = self.kinematics.direct_kin(valid_ik_solutions[0], True)
                #print "dk_pos: [%.4f; %.4f; %.4f]" % (dk_pos[0],dk_pos[1],dk_pos[2])
                self.send_vrep_joint_targets(valid_ik_solutions[0], True)
                #tmp = self.kinematics.offset2world(valid_ik_solutions[0])
                #send tmp to GUI

	
	    #self.send_status2gui( status.STATUS_NODE_NO_ERROR, status.STATUS_VREP_WAITING_4_CMD, "movement in progress")
	
	
        #rospy.logwarn('self.poly_y == 0')

        self.lock.release()

        
    def callback_vrep_joint_states(self,msg):
        if(not self.run_status):
            return

        self.lock.acquire()
        self.vrep_msg_name = msg.name
        self.vrep_msg_position = msg.position
        self.vrep_msg_velocity = msg.velocity
        self.vrep_msg_effort = msg.effort
        self.lock.release()


    #returns (found, position, velocity, effort) for the joint joint_name
    #(found is 1 if found, 0 otherwise)
    def return_joint_state(self, joint_name):

         #no messages yet
        if self.vrep_msg_name == []:
            rospy.logerr("No robot_state messages received!\n")
            return (0, 0., 0., 0.)

        #return info for this joint
        self.lock.acquire()
        if joint_name in self.vrep_msg_name:
            index = self.vrep_msg_name.index(joint_name)
            position = self.vrep_msg_position[index]
            velocity = self.vrep_msg_velocity[index]
            effort = self.vrep_msg_effort[index]

        #unless it's not found
        else:
            rospy.logerr("Joint %s not found!", (joint_name,))
            self.lock.release()
            return (0, 0., 0., 0.)
        self.lock.release()
        return (1, position, velocity, effort)


         #server callback: returns arrays of position, velocity, and effort
         #for a list of joints specified by name
    #def return_joint_states(self, req):
    #    joints_found = []
    #    positions = []
    #    velocities = []
    #    efforts = []
    #    for joint_name in req.name:
    #        (found, position, velocity, effort) = self.return_joint_state(joint_name)
    #        joints_found.append(found)
    #        positions.append(position)
    #        velocities.append(velocity)
    #        efforts.append(effort)
    #    #return ReturnJointStatesResponse(joints_found, positions, velocities, efforts)
        
        
    def parse_input_from_gui(self, msg):

        #CONFIG
        self.config_fontsize = int(msg.Fontsize)
        self.config_pencil_length = float(msg.PencilLength)
        self.config_use_thetas = int(msg.UseThetas)
        self.config_thetas = np.array([msg.Theta_1,
            msg.Theta_2,
            msg.Theta_3,
            msg.Theta_4,
            msg.Theta_5])

        self.config_use_pos = int(msg.UsePos)
        self.config_pos = np.array([msg.Pos_X,
            msg.Pos_Y,
            msg.Pos_Z])

        #DATA
        self.data_string = msg.letters
    
    def send_status2gui(self, nodestatus, vrepstatus, error_txt):
        msg=rw_node_state()
        msg.nodestatus=nodestatus
        msg.vrepstatus=vrepstatus
        msg.error=error_txt
        self.status = nodestatus
        self.status_vrep = vrepstatus
        self.status_string = error_txt
        #set status: [nodestatus;vrepstatus;]
        self.pub2gui.publish(msg)
        
        rospy.loginfo("send status 2 gui")
        #rospy.loginfo(commandStr)
        
        
        
if __name__ == '__main__':
    Node()
