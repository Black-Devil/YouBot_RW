#!/usr/bin/env python2

#from time import sleep, time
import time
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

import sync


class Node(object):
    def __init__(self):
        """ init main ros node of youbot remote writing

        @return <b><i><c> [void]: </c></i></b> nothing
        """

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
        
        rospy.Subscriber('/youbot_rw/gui2node', rw_node, self.callback_process_cmd)
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
        """ init parameters for main node of youbot remote writing

        @return <b><i><c> [void]: </c></i></b> nothing
        """
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
        self.config_thetas_bogen = np.array([0.0,
            0.0,
            0.0,
            0.0,
            0.0])
        self.config_cur_pos = np.array([np.nan,
            np.nan,
            np.nan])
        self.config_trgt_pos = np.array([np.nan,
            np.nan,
            np.nan])
        self.init_pos = False

        # parse xml letter database
        script_path = os.path.dirname(os.path.abspath(__file__))
        print("script dir: "), script_path
        tmp_use_rosparam_path = rospy.get_param("/youbot_rw_node/general/USE_ROSPARAM_PATH_FOR_DATABASE")
        if(tmp_use_rosparam_path):
            db_path = rospy.get_param("/youbot_rw_node/general/PATH_OF_LETTER_DATABASE")
        else:
            db_path = script_path + '/../../../../../material/YouBot_RW_Material/Buchstaben_Datenbank/letter_database.xml'

        print("opening following letter_database: "), db_path
        self.letter_database = ET.parse(db_path)
        self.ldb_root = self.letter_database.getroot()

        # print loaded ldb elements
        print("Loaded "), self.ldb_root.tag, (": ")
        for letter in self.ldb_root:
            print(letter.tag)


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
        #time.sleep(0.01)
    
    
     
    def callback_process_cmd(self,msg):
        if(not self.run_status):
            return

        self.lock.acquire()

        rospy.loginfo("process_cmd callback")
        self.time = rospy.get_time()
        self.parse_input_from_gui(msg)
        self.send_status2gui( status.STATUS_NODE_NO_ERROR, status.STATUS_VREP_WAITING_4_CMD, "received write command")

	    #DO WRITING WITH ROBOT HERE
        if(self.config_processMode == status.PROCESSING_MODE_WRITING):
            if(self.init_pos == False):
                print("WARNING - programm is not yet initialized complete! The robot position is unknown! Therefor a ptp movement to position [0,0,0] is processed first.")
                tmp = self.config_trgt_pos
                self.config_trgt_pos = np.array([0.0, 0.0, 0.1])
                self.process_ptp_position()
                self.config_trgt_pos = tmp
                self.process_writing()
            else:
                self.process_writing()
        elif(self.config_processMode == status.PROCESSING_MODE_LOGO):
            print("triggered logo")
            # TODO: implement Logo

        elif(self.config_processMode == status.PROCESSING_MODE_PTP_ANGLES):
            self.process_ptp_angles()
        elif(self.config_processMode == status.PROCESSING_MODE_LIN_ANGLES):
            print("ERROR - linear movement through angle input is not implemented yet")
            self.send_status2gui( status.STATUS_NODE_NO_ERROR, status.STATUS_VREP_WAITING_4_CMD, "ERROR - linear movement through angle input is not implemented yet")
        elif(self.config_processMode == status.PROCESSING_MODE_PTP_POSITION):
            self.process_ptp_position()
            print("pos: [%.4f; %.4f; %.4f]" % (self.config_cur_pos[0],self.config_cur_pos[1],self.config_cur_pos[2]) )
        elif(self.config_processMode == status.PROCESSING_MODE_LIN_POSITION):
            if(self.init_pos == False):
                print("WARNING - programm is not yet initialized complete! The robot position is unknown! Therefor a ptp movement to position [0,0,0] is processed first.")
                tmp = self.config_trgt_pos
                self.config_trgt_pos = np.array([0.0, 0.0, 0.0])
                self.process_ptp_position()
                self.config_trgt_pos = tmp
                self.process_lin_position()
            else:
                self.process_lin_position()


	    #self.send_status2gui( status.STATUS_NODE_NO_ERROR, status.STATUS_VREP_WAITING_4_CMD, "movement in progress")

        #rospy.logwarn('self.poly_y == 0')

        self.lock.release()


    def process_ptp_angles(self):
        print("triggered angles")
        #tmp = self.kinematics.offset2world(self.config_thetas_bogen)
        if(self.kinematics.isSolutionValid(self.config_thetas_bogen)):
            self.send_vrep_joint_targets(self.config_thetas_bogen, True)
            self.config_cur_pos = self.kinematics.direct_kin(self.config_thetas_bogen)
            self.init_pos = True
            print("pos: [%.4f; %.4f; %.4f]" % (self.config_cur_pos[0],self.config_cur_pos[1],self.config_cur_pos[2]) )
            pos_wp = self.kinematics.direct_kin_2_wristPoint(self.config_thetas_bogen)
            print("pos_wp: [%.4f; %.4f; %.4f]" % (pos_wp[0],pos_wp[1],pos_wp[2]) )
            self.send_status2gui( status.STATUS_NODE_NO_ERROR, status.STATUS_VREP_WAITING_4_CMD, "angle processing done")
        else:
            print("ERROR: Target angles are not valid! This should never happen, if GUI is used.")


    def process_lin_position(self):
        print("== triggered lin position ==")
        tmpPos = list()
        tmpPos.append( np.array([self.config_trgt_pos[0],  self.config_trgt_pos[1],  self.config_trgt_pos[2]]) )
        self.process_linear_movement(tmpPos, False, True)
        #self.process_linear_movement_with_angles(tmpPos, True, True)
        print("== lin position movement done==")

    def process_ptp_position(self):
        print("== triggered ptp position ==")
        #time.sleep(0.2)

        tmpPos = np.matrix((self.config_trgt_pos[0],  self.config_trgt_pos[1],  self.config_trgt_pos[2])).transpose()
        #print "input ik", tmpPos
        valid_ik_solutions = self.kinematics.get_valid_inverse_kin_solutions(tmpPos, True, False, False)
        #print("debug get valid ik solutions")
        #time.sleep(0.2)

        valid_sol = False
        if not valid_ik_solutions:
            # try again without fast calculation
            valid_ik_solutions = self.kinematics.get_valid_inverse_kin_solutions(tmpPos, False, False, False)
            #print("debug get valid ik solutions again")
            #time.sleep(0.2)

            if not valid_ik_solutions:
                print "// no valid ik solution possible! //"
                self.send_status2gui( status.STATUS_NODE_NO_ERROR, status.STATUS_VREP_WAITING_4_CMD, "position processing not possible, found no ik solution")
            else:
                valid_sol = True
        else:
            valid_sol = True

        if valid_sol:
            print "// valid ik solution possible! //"
            #print "first valid ik solution: [%.4f; %.4f; %.4f; %.4f; %.4f;]" % (math.degrees(valid_ik_solutions[0][0]), math.degrees(valid_ik_solutions[0][1]), math.degrees(valid_ik_solutions[0][2]), math.degrees(valid_ik_solutions[0][3]), math.degrees(valid_ik_solutions[0][4]) ) , self.kinematics.isSolutionValid(valid_ik_solutions[0])
            #dk_pos = self.kinematics.direct_kin(valid_ik_solutions[0], True)
            #print "dk_pos: [%.4f; %.4f; %.4f]" % (dk_pos[0],dk_pos[1],dk_pos[2])

            #print("debug solutions:"),valid_ik_solutions

            self.send_vrep_joint_targets(valid_ik_solutions[0], True)
            self.config_cur_pos = self.kinematics.direct_kin(valid_ik_solutions[0])
            self.init_pos = True
            self.config_thetas_bogen = valid_ik_solutions[0]


            self.config_thetas[0] = math.degrees(self.config_thetas_bogen[0])
            self.config_thetas[1] = math.degrees(self.config_thetas_bogen[1])
            self.config_thetas[2] = math.degrees(self.config_thetas_bogen[2])
            self.config_thetas[3] = math.degrees(self.config_thetas_bogen[3])
            self.config_thetas[4] = math.degrees(self.config_thetas_bogen[4])

            self.send_status2gui( status.STATUS_NODE_NO_ERROR, status.STATUS_VREP_WAITING_4_CMD, "position processing done, found solution")
            #tmp = self.kinematics.offset2world(valid_ik_solutions[0])
            #send tmp to GUI
            print("== ptp position movement done==")


    def process_writing(self):
        print("== triggered writing ==")
        # all letter coorinates are in font_size 100(100mm high)
        self.base_fs = 100.0
        self.centimeter2meter_factor = 0.01
        self.letter_size_factor = float(self.config_fontsize)/self.base_fs

        self.between_letter_margin = self.letter_size_factor * 0.01#0.001  # x coordinate
        self.between_line_margin = self.letter_size_factor * 0.03    # y coordinate
        self.letter_height = self.config_fontsize * 0.001             # height of written letters in m

        self.hoverOffset = 0.01             # z coordinate
        self.line_ending_threshold = 0.1
        self.start_line_offset = -0.00


        # set start position of writing in write_plane coordinates
        current_write_pos = np.array([ self.start_line_offset, -self.line_ending_threshold, self.hoverOffset ])    # is always in hoveroffset

        #print("start write pos: "), current_write_pos

        # at the start of writing go to the start position in linear movement without singularity checking to prevent collisions
        self.process_linear_movement( np.array([ current_write_pos ]), True, False)

        print("moved to start write pos: [%.4f; %.4f; %.4f]" % (self.config_cur_pos[0],self.config_cur_pos[1],self.config_cur_pos[2]) )


        for letter in self.data_string:
            # search letter in database
            pointlist = self.get_pointlist4letter(letter)
            if not pointlist:
                print(letter), ("WTF! THIS SHOULD NEVER HAPPEN!")
            else:
                print("raw pointlist: "), pointlist

                # transform pointlist to current write position     # let out z coordinate
                max_y_of_pointlist = current_write_pos[1]
                for point in pointlist:
                    point[0] = float(point[0]) + current_write_pos[0]
                    point[1] = float(point[1]) + current_write_pos[1]
                    if(point[1] > max_y_of_pointlist):
                        max_y_of_pointlist = point[1]

                # move from current position to next write position with hoveroffset and then to the write plane
                toNextLetter = np.array([ np.array([ current_write_pos[0], current_write_pos[1], current_write_pos[2]]),
                                          np.array([ pointlist[0][0], pointlist[0][1], self.hoverOffset ]),
                                          np.array([ pointlist[0][0], pointlist[0][1], pointlist[0][2] ]) ])
                print("toNextLetter: "), toNextLetter
                self.process_linear_movement(toNextLetter, True, True)

                # transform pointlist to current position
                print("transformed pointlist: "), pointlist
                self.process_linear_movement(pointlist, True, True)

                # move to hover position
                toHoverPos = np.array([ np.array([ self.config_cur_pos[0], self.config_cur_pos[1], (self.config_cur_pos[2] + self.hoverOffset) ])  ])
                print("toHoverPos: "), toHoverPos
                self.process_linear_movement(toHoverPos, True, True)

                # update current write position
                current_write_pos = np.array([ self.config_cur_pos[0], self.config_cur_pos[1], self.config_cur_pos[2] ])

                # before writing the next letter put in a margin after most right position of letter
                current_write_pos[1] = max_y_of_pointlist + self.between_letter_margin

                #check for line ending
                if(current_write_pos[1] > self.line_ending_threshold):
                    current_write_pos[0] = current_write_pos[0] + self.letter_height + self.between_line_margin
                    current_write_pos[1] = -self.line_ending_threshold

                print("== letter "), letter, (" done ==")

        print("== writing done ==")


    def calc_current_tcp_position(self):
        joint_thetas = sync.getJointPostition()
        self.config_thetas_bogen = np.array([joint_thetas[0],
            joint_thetas[1],
            joint_thetas[2],
            joint_thetas[3],
            joint_thetas[4]])
        self.config_thetas[0] = math.degrees(self.config_thetas_bogen[0])
        self.config_thetas[1] = math.degrees(self.config_thetas_bogen[1])
        self.config_thetas[2] = math.degrees(self.config_thetas_bogen[2])
        self.config_thetas[3] = math.degrees(self.config_thetas_bogen[3])
        self.config_thetas[4] = math.degrees(self.config_thetas_bogen[4])

        self.config_cur_pos = self.kinematics.direct_kin(self.config_thetas_bogen)

    def get_pointlist4letter(self, letter):
        print("get pointlist for letter: "), letter

        # convert lowercase letters to upper case letters
        letter = letter.upper()
        dbElement = self.ldb_root.find(letter)
        resPointlist = list()
        if(dbElement == None):
            dbElement = self.ldb_root.find("NOT_IMPL")
            print("WARNING: no element found with tag %s!") %(letter)

        if(dbElement != None):
            dbPointlist = list(dbElement)
            isHovering = False
            for point in dbPointlist:

                # handle hover from last point
                if(isHovering):
                    isHovering = False
                    resPointlist.append(np.array([ float(point.attrib['x'])*self.letter_size_factor*self.centimeter2meter_factor, float(point.attrib['y'])*self.letter_size_factor*self.centimeter2meter_factor, self.hoverOffset ])  )

                # append point coordinates
                resPointlist.append(np.array([ float(point.attrib['x'])*self.letter_size_factor*self.centimeter2meter_factor, float(point.attrib['y'])*self.letter_size_factor*self.centimeter2meter_factor, float(point.attrib['z'])*self.letter_size_factor*self.centimeter2meter_factor ])  )

                # handle hover to next point
                if( int(point.attrib['hov2nxt']) == 1 ):
                    print("= HOVER2NEXT =")
                    isHovering = True
                    resPointlist.append(np.array([ float(point.attrib['x'])*self.letter_size_factor*self.centimeter2meter_factor, float(point.attrib['y'])*self.letter_size_factor*self.centimeter2meter_factor, self.hoverOffset ])  )

        else:
            print("ERROR: no element found with tag %s!") %(letter)

        return resPointlist


    def process_linear_movement(self, point_list, limit_solution, check_singularities):
        """ takes np.array of 3d points(np.matrix) and processes linear movement from current position to all points

        :param todo
        :type todo
        :returns: todo
        :rtype: todo
        """
        # calc step size
        step_size = 0.002
        #max_step = int(1.0/step_size)

        # init on current angles
        last_angles = self.config_thetas_bogen
        last_condition = -1

        # process pointlist
        for i in point_list:
            origin = self.config_cur_pos
            print("lin_move from: "), origin, (" to: "), i

            move_vec = np.array([ i[0] - origin[0], i[1] - origin[1], i[2] - origin[2] ])
            move_length = np.sqrt(move_vec[0]**2 + move_vec[1]**2 + move_vec[2]**2)

            step_count_int = int(move_length/step_size) + 1
            step_count_float = move_length/step_size
            step_vec = np.array([ move_vec[0] / step_count_float, move_vec[1] / step_count_float, move_vec[2] / step_count_float ])
            #print("current point: "), i
            #print("stepcount: "), step_count_int

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
                valid_ik_solutions, valid_ik_solutions_condition = self.kinematics.get_valid_inverse_kin_solutions(current_trgt, True, limit_solution, True)
                if not valid_ik_solutions:
                    #try again without fast calculation
                    valid_ik_solutions, valid_ik_solutions_condition = self.kinematics.get_valid_inverse_kin_solutions(current_trgt, False, limit_solution, True)
                    if not valid_ik_solutions:
                        print("Found no ik solution for point(%.4f; %.4f; %.4f). Processing goes on with next point.") %(current_trgt[0], current_trgt[1], current_trgt[2])
                    else:
                        current_valid = True
                else:
                    current_valid = True

                if current_valid:

                    # TODO: check for singularities in angles
                    # search solutions without singularities
                    if(check_singularities):
                        valid_sol_nosing = list()
                        valid_sol_nosing_cond = list()
                        int_counter = -1
                        for s in valid_ik_solutions:
                            int_counter = int_counter + 1
                            has_sing = False
                            if(last_condition != -1):
                                if( (s[0] > 0.0 and last_angles[0] < 0.0) or (s[0] < 0.0 and last_angles[0] > 0.0)): has_sing = True
                                if( (s[1] > 0.0 and last_angles[1] < 0.0) or (s[1] < 0.0 and last_angles[1] > 0.0)): has_sing = True
                                if( (s[2] > 0.0 and last_angles[2] < 0.0) or (s[2] < 0.0 and last_angles[2] > 0.0)): has_sing = True
                                if( (s[3] > 0.0 and last_angles[3] < 0.0) or (s[3] < 0.0 and last_angles[3] > 0.0)): has_sing = True

                            if not has_sing:
                                valid_sol_nosing.append(s)
                                valid_sol_nosing_cond.append(valid_ik_solutions_condition[int_counter])

                        # if singularity is not preventable, hover and go to new condition
                        if not valid_sol_nosing:
                            print("!!! SINGULARITY happened !!!")
                            best_sol = valid_ik_solutions[0]
                            last_condition = valid_ik_solutions_condition[0]
                        else:
                            best_sol, last_condition = self.get_best_sol_through_condition(last_condition, valid_sol_nosing, valid_sol_nosing_cond)
                            #print("cur_condition:"), last_condition
                            # TODO: if condition changes, handle this

                    else:
                        best_sol = valid_ik_solutions[0]
                    
                    # do the movement
                    self.send_vrep_joint_targets(best_sol, True)
                    self.config_cur_pos = np.array([ i[0], i[1], i[2] ])
                    self.init_pos = True
                    self.config_thetas_bogen = best_sol

                    last_angles = best_sol

                    self.config_thetas[0] = math.degrees(self.config_thetas_bogen[0])
                    self.config_thetas[1] = math.degrees(self.config_thetas_bogen[1])
                    self.config_thetas[2] = math.degrees(self.config_thetas_bogen[2])
                    self.config_thetas[3] = math.degrees(self.config_thetas_bogen[3])
                    self.config_thetas[4] = math.degrees(self.config_thetas_bogen[4])
                    # TODO: sending status to gui here produces sometimes memory errors (WTF!)
                    #self.send_status2gui( status.STATUS_NODE_NO_ERROR, status.STATUS_VREP_WAITING_4_CMD, "linear movement in progress...")
                    # sleep a moment to prevent unsynchronization
                    #tmp_wait = 0.1
                    #print("wait")
                    #time.sleep(tmp_wait)     # sleep 0.1 sec
                    #print("waited for "), tmp_wait, ("sec")

            #self.config_current_pos = np.array([ i[0], i[1], i[2] ])
            self.send_status2gui( status.STATUS_NODE_NO_ERROR, status.STATUS_VREP_WAITING_4_CMD, "linear movement in progress...")



    def process_linear_movement_with_angles(self, point_list, limit_solution, check_singularities):
        """ takes np.array of 3d points(np.matrix) and processes linear movement from current position to all points

        pointlist--> best solution --> interpolate angles between current position and best solution

        :param todo
        :type todo
        :returns: todo
        :rtype: todo
        """
        # calc step size
        step_size = 0.002
        #max_step = int(1.0/step_size)

        # init on current angles
        last_angles = self.config_thetas_bogen
        last_condition = -1

        # process pointlist
        for i in point_list:
            origin = self.config_cur_pos
            origin_angles = self.config_thetas_bogen
            print("lin_move from: "), origin, (" to: "), i

            move_vec = np.array([ i[0] - origin[0], i[1] - origin[1], i[2] - origin[2] ])
            move_length = np.sqrt(move_vec[0]**2 + move_vec[1]**2 + move_vec[2]**2)

            step_count_int = int(move_length/step_size) + 1


            current_valid = False
            #print k


            # calc inverse kinematik for current point
            valid_ik_solutions, valid_ik_solutions_condition = self.kinematics.get_valid_inverse_kin_solutions(i, True, limit_solution, True)
            if not valid_ik_solutions:
                #try again without fast calculation
                valid_ik_solutions, valid_ik_solutions_condition = self.kinematics.get_valid_inverse_kin_solutions(i, False, limit_solution, True)
                if not valid_ik_solutions:
                    print("Found no ik solution for point(%.4f; %.4f; %.4f). Processing goes on with next point.") %(i[0], i[1], i[2])
                else:
                    current_valid = True
            else:
                current_valid = True

            if current_valid:
                # search solutions without singularities
                if(check_singularities):
                    valid_sol_nosing = list()
                    valid_sol_nosing_cond = list()
                    int_counter = -1
                    for s in valid_ik_solutions:
                        int_counter = int_counter + 1
                        has_sing = False
                        if(last_condition != -1):
                            if( (s[0] > 0.0 and last_angles[0] < 0.0) or (s[0] < 0.0 and last_angles[0] > 0.0)): has_sing = True
                            if( (s[1] > 0.0 and last_angles[1] < 0.0) or (s[1] < 0.0 and last_angles[1] > 0.0)): has_sing = True
                            if( (s[2] > 0.0 and last_angles[2] < 0.0) or (s[2] < 0.0 and last_angles[2] > 0.0)): has_sing = True
                            if( (s[3] > 0.0 and last_angles[3] < 0.0) or (s[3] < 0.0 and last_angles[3] > 0.0)): has_sing = True

                        if not has_sing:
                            valid_sol_nosing.append(s)
                            valid_sol_nosing_cond.append(valid_ik_solutions_condition[int_counter])

                    # if singularity is not preventable, hover and go to new condition
                    if not valid_sol_nosing:
                        print("!!! SINGULARITY happened !!!")
                        best_sol = valid_ik_solutions[0]
                        last_condition = valid_ik_solutions_condition[0]
                    else:
                        best_sol, last_condition = self.get_best_sol_through_condition(last_condition, valid_sol_nosing, valid_sol_nosing_cond)
                        #print("cur_condition:"), last_condition
                        # TODO: if condition changes, handle this

                else:
                    best_sol = valid_ik_solutions[0]

                # calc steps for anggles
                angle_steps = list()
                tmp_int_counter = -1
                for a in self.config_thetas:
                    tmp_int_counter = tmp_int_counter +1
                    angle_steps.append( (best_sol[tmp_int_counter] - a) / step_count_int)

                # interpolate
                for k in range(1,step_count_int+1):
                    if(k == step_count_int):
                        #print("debug k = "),int(1.0/step_size)+1
                        current_trgt_angles = best_sol
                    else:
                        current_trgt_angles = np.array([ origin_angles[0] + k*angle_steps[0], origin_angles[1] + k*angle_steps[1], origin_angles[2] + k*angle_steps[2],
                                                    origin_angles[3] + k*angle_steps[3], origin_angles[4] + k*angle_steps[4] ])

                    # do the movement
                    self.send_vrep_joint_targets(current_trgt_angles, True)
                    self.config_cur_pos = np.array([ i[0], i[1], i[2] ])
                    self.init_pos = True
                    self.config_thetas_bogen = current_trgt_angles

                    last_angles = current_trgt_angles

                    self.config_thetas[0] = math.degrees(self.config_thetas_bogen[0])
                    self.config_thetas[1] = math.degrees(self.config_thetas_bogen[1])
                    self.config_thetas[2] = math.degrees(self.config_thetas_bogen[2])
                    self.config_thetas[3] = math.degrees(self.config_thetas_bogen[3])
                    self.config_thetas[4] = math.degrees(self.config_thetas_bogen[4])

            #self.config_current_pos = np.array([ i[0], i[1], i[2] ])
            self.send_status2gui( status.STATUS_NODE_NO_ERROR, status.STATUS_VREP_WAITING_4_CMD, "linear movement in progress...")


    def get_best_sol_through_condition(self, last_condition, solutions, solutions_cond):
        if not solutions:
            print("WARNING: solutions is empty")
            return -1, -1

        if(last_condition == -1):
            # take first solution
            return solutions[0], solutions_cond[0]

        else:
            # take best solution
            int_counter = -1
            for s in solutions_cond:
                int_counter = int_counter + 1
                if(s == last_condition):
                    return solutions[int_counter], s
            print("WARNING: condition changed in this step")
            return solutions[0], solutions_cond[0]


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

        self.config_thetas =  self.kinematics.offset2world(self.config_thetas)
        self.config_thetas_bogen = np.array([math.radians(self.config_thetas[0]),
            math.radians(self.config_thetas[1]),
            math.radians(self.config_thetas[2]),
            math.radians(self.config_thetas[3]),
            math.radians(self.config_thetas[4]) ])
        #print("parse input: "), self.config_thetas
        self.config_trgt_pos = np.array([msg.Pos_X,
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
        msg.Pos_X = self.config_cur_pos[0]
        msg.Pos_Y = self.config_cur_pos[1]
        msg.Pos_Z = self.config_cur_pos[2]
        tmp = self.kinematics.offset2world(self.config_thetas)
        msg.Theta_1 = tmp[0]
        msg.Theta_2 = tmp[1]
        msg.Theta_3 = tmp[2]
        msg.Theta_4 = tmp[3]
        msg.Theta_5 = tmp[4]

        self.status = nodestatus
        self.status_vrep = vrepstatus
        self.status_string = error_txt
        #set status: [nodestatus;vrepstatus;]
        self.pub2gui.publish(msg)
        
        #rospy.loginfo("send status 2 gui")
        #rospy.loginfo(commandStr)
        
        
        
if __name__ == '__main__':
    Node()
