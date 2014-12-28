#!/usr/bin/env python

from time import sleep, time
import math
import threading

import rospy
from std_msgs.msg import *
from sensor_msgs.msg import *

import numpy as np

import kinematics_geom as kin

from lxml import etree

import xml.etree.ElementTree as ET

import status_intf as status
from youbot_rw_rqt_gui.msg import *

import os


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
        self.config_thetas = np.array([0.0,
            0.0,
            0.0,
            0.0,
            0.0])
        self.config_pos = np.array([0.0,
            0.0,
            0.0])

        # parse xml letter database
        script_path = os.path.dirname(os.path.abspath(__file__))
        print("script dir: "), script_path
        db_path = script_path + '/../../../../../material/YouBot_RW_Material/Buchstaben_Datenbank/letter_database.xml'
        print("opening following letter_database: "), db_path
        self.letter_database = ET.parse(db_path)
        self.ldb_root = self.letter_database.getroot()

        # print loaded ldb elements
        print("Loaded "), self.ldb_root.tag, (": ")
        for letter in self.ldb_root:
            pointlist = self.get_pointlist4letter(letter.tag)
            print(letter.tag),(": "), pointlist


        #do init here


    def spin_restarter(self):
        #print "waiting for button"
        while(not rospy.is_shutdown() and self.run):
            if(self.run_status):
                # TODO: is this needed?
                #self.init_params()
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
        #rospy.loginfo("send vrep joint targets")
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
        if(self.config_processMode == status.PROCESSING_MODE_WRITING):
            self.process_writing()

        elif(self.config_processMode == status.PROCESSING_MODE_LOGO):
            print("triggered logo")
            # TODO: implement Logo

        elif(self.config_processMode == status.PROCESSING_MODE_PTP_ANGLES or self.config_processMode == status.PROCESSING_MODE_LIN_ANGLES):
            print("triggered angles")
            tmp = self.kinematics.offset2world(self.config_thetas)
            if(self.kinematics.isSolutionValid(tmp)):
                self.send_vrep_joint_targets(tmp, False)
                self.config_pos = self.kinematics.direct_kin(np.deg2rad(tmp))
                print("pos: [%.4f; %.4f; %.4f]" % (self.config_pos[0],self.config_pos[1],self.config_pos[2]) )
                pos_wp = self.kinematics.direct_kin_2_wristPoint(np.deg2rad(tmp))
                print("pos_wp: [%.4f; %.4f; %.4f]" % (pos_wp[0],pos_wp[1],pos_wp[2]) )
                self.send_status2gui( status.STATUS_NODE_NO_ERROR, status.STATUS_VREP_WAITING_4_CMD, "angle processing done")
            else:
                print("ERROR: Target angles are not valid! This should never happen, if GUI is used.")

        elif(self.config_processMode == status.PROCESSING_MODE_PTP_POSITION or self.config_processMode == status.PROCESSING_MODE_LIN_POSITION):
            print("triggered position")
            tmpPos = np.matrix((self.config_pos[0],  self.config_pos[1],  self.config_pos[2])).transpose()
            #print "input ik", tmpPos
            valid_ik_solutions = self.kinematics.get_valid_inverse_kin_solutions(tmpPos, True)
            valid_sol = False
            if not valid_ik_solutions:
                # try again without fast calculation
                valid_ik_solutions = self.kinematics.get_valid_inverse_kin_solutions(tmpPos, False)
                if not valid_ik_solutions:
                    print "// no valid ik solution possible! //"
                    self.send_status2gui( status.STATUS_NODE_NO_ERROR, status.STATUS_VREP_WAITING_4_CMD, "position processing done, found no solution")
                else:
                    valid_sol = True
            else:
                valid_sol = True

            if valid_sol:
                print "// valid ik solution possible! //"
                #print "first valid ik solution: [%.4f; %.4f; %.4f; %.4f; %.4f;]" % (math.degrees(valid_ik_solutions[0][0]), math.degrees(valid_ik_solutions[0][1]), math.degrees(valid_ik_solutions[0][2]), math.degrees(valid_ik_solutions[0][3]), math.degrees(valid_ik_solutions[0][4]) ) , self.kinematics.isSolutionValid(valid_ik_solutions[0])
                #dk_pos = self.kinematics.direct_kin(valid_ik_solutions[0], True)
                #print "dk_pos: [%.4f; %.4f; %.4f]" % (dk_pos[0],dk_pos[1],dk_pos[2])
                self.config_thetas = valid_ik_solutions[0]
                self.send_vrep_joint_targets(valid_ik_solutions[0], True)
                self.send_status2gui( status.STATUS_NODE_NO_ERROR, status.STATUS_VREP_WAITING_4_CMD, "position processing done, found solution")
                #tmp = self.kinematics.offset2world(valid_ik_solutions[0])
                #send tmp to GUI

	
	    #self.send_status2gui( status.STATUS_NODE_NO_ERROR, status.STATUS_VREP_WAITING_4_CMD, "movement in progress")
	
	
        #rospy.logwarn('self.poly_y == 0')

        self.lock.release()


    def process_writing(self):
        print("triggered writing")
        # TODO: implement Writing
        for letter in self.data_string:
            # search letter in database
            pointlist = self.get_pointlist4letter(letter)
            if not pointlist:
                print(letter), ("is not yet in the letter database and will be skipt during writing process")
            else:
                # TODO: implement transformation handling for the individual letters, etc.
                hoverOffset = 0.01
                # move from current position to next letter position
                toNextLetter = np.array([ np.matrix([self.config_pos[0], self.config_pos[1], self.config_pos[2]]).transpose(),
                                            np.matrix([float(pointlist[0][0]), float(pointlist[0][1]), (float(pointlist[0][2]) + hoverOffset)]).transpose(),
                                            np.matrix([float(pointlist[0][0]), float(pointlist[0][1]), float(pointlist[0][2]) ]).transpose() ])
                self.process_linear_movement(toNextLetter)

                self.process_linear_movement(pointlist)

                # move to hover position
                toHoverPos = np.array([ np.matrix([float(self.config_pos[0]), float(self.config_pos[1]), float(self.config_pos[2])]).transpose(),
                                            np.matrix([float(self.config_pos[0]), float(self.config_pos[1]), (float(self.config_pos[2]) + hoverOffset)]).transpose()    ])

                self.process_linear_movement(toHoverPos)


        #debugPointList = np.array([ np.matrix([0.05, 0.0, 0.05]).transpose(),
        #                            np.matrix([-0.05, 0.0, 0.05]).transpose(),
        #                            np.matrix([0.05, 0.0, 0.1]).transpose() ])
        #self.process_linear_movement(debugPointList)


    def get_pointlist4letter(self, letter):
        dbElement = self.ldb_root.find(letter)
        resPointlist = list()
        if(dbElement != None):
            dbPointlist = list(dbElement)
            for point in dbPointlist:
                resPointlist.append(np.matrix([ float(point.attrib['x']), float(point.attrib['y']), float(point.attrib['z']) ]).transpose() )
                #print(" [%f; %f; %f]") %(float(point.attrib['x']), float(point.attrib['y']), float(point.attrib['z'])), letter
        else:
            print("ERROR: no element found with tag %s!") %(letter)

        return resPointlist


    def process_linear_movement(self, point_list):
        """ takes np.array of 3d points(np.matrix) and processes linear movement from current position to all points

        :param todo
        :type todo
        :returns: todo
        :rtype: todo
        """
        # calc step size
        step_size = 0.002
        #max_step = int(1.0/step_size)

        # process pointlist
        for i in point_list:
            origin = self.config_pos
            move_vec = np.array([ i[0] - origin[0], i[1] - origin[1], i[2] - origin[2] ])
            move_length = np.sqrt(move_vec[0]**2 + move_vec[1]**2 + move_vec[2]**2)

            step_count_int = int(move_length/step_size) + 1
            step_count_float = move_length/step_size
            step_vec = np.array([ move_vec[0] / step_count_float, move_vec[1] / step_count_float, move_vec[2] / step_count_float ])
            #print("current point: "), i

            # process single point out of pointlist
            for k in range(1,step_count_int+1):

                current_valid = False
                #print k
                if(k == step_count_int):
                    #print("debug k = "),int(1.0/step_size)+1
                    current_trgt = np.array([ i[0], i[1], i[2]])
                else:
                    current_trgt = np.array([ origin[0] + k*step_vec[0], origin[1] + k*step_vec[1], origin[2] + k*step_vec[2]])

                # calc inverse kinematik for current point
                valid_ik_solutions = self.kinematics.get_valid_inverse_kin_solutions(current_trgt, True)
                if not valid_ik_solutions:
                    #try again without fast calculation
                    valid_ik_solutions = self.kinematics.get_valid_inverse_kin_solutions(current_trgt, False)
                    if not valid_ik_solutions:
                        print("Found no ik solution for point(%.4f; %.4f; %.4f). Processing goes on with next point.") %(current_trgt[0], current_trgt[1], current_trgt[2])
                    else:
                        current_valid = True
                else:
                    current_valid = True

                if current_valid:
                    # TODO: take best solution
                    self.config_thetas = valid_ik_solutions[0]
                    self.send_vrep_joint_targets(valid_ik_solutions[0], True)

            self.config_pos = np.array([ i[0], i[1], i[2] ])
            self.send_status2gui( status.STATUS_NODE_NO_ERROR, status.STATUS_VREP_WAITING_4_CMD, "linear movement in progress...")



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
        self.config_processMode = int(msg.processmode)
        self.config_thetas = np.array([msg.Theta_1,
            msg.Theta_2,
            msg.Theta_3,
            msg.Theta_4,
            msg.Theta_5])
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

        # setting GUI output
        msg.Pos_X = self.config_pos[0]
        msg.Pos_Y = self.config_pos[1]
        msg.Pos_Z = self.config_pos[2]
        msg.Theta_1 = self.config_thetas[0]
        msg.Theta_2 = self.config_thetas[1]
        msg.Theta_3 = self.config_thetas[2]
        msg.Theta_4 = self.config_thetas[3]
        msg.Theta_5 = self.config_thetas[4]

        self.status = nodestatus
        self.status_vrep = vrepstatus
        self.status_string = error_txt
        #set status: [nodestatus;vrepstatus;]
        self.pub2gui.publish(msg)
        
        #rospy.loginfo("send status 2 gui")
        #rospy.loginfo(commandStr)
        
        
        
if __name__ == '__main__':
    Node()
