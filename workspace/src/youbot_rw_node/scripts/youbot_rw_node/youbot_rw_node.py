#!/usr/bin/env python

from time import sleep, time
#import math
import threading

import rospy
from std_msgs.msg import *

import robot_config as rc
import KinematicFunctions as kf
import MotionFunctions as mf


import status_intf as status

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
        
        rospy.Subscriber('/youbot_rw/gui2node', String, self.callback_write_cmd)
        self.pub2gui = rospy.Publisher('/youbot_rw/node2gui', String, tcp_nodelay=True,queue_size=1)
        
        
        self.spin_restarter()        
        
    def init_params(self):
        self.rate = 100
        self.disable = False
        self.status = 1 #0= error 1= no error
        self.status_string = "no error"
        self.status_vrep = 0 #0=doing nothing 1= in progress 2= movement done
                
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
        
        
        
    def callback_write_cmd(self,msg):
        if(not self.run_status):
            return

        self.lock.acquire()


        rospy.loginfo("write_cmd callback")        
        self.time = rospy.get_time()
        self.parse_input_from_gui(msg)
        self.send_status2gui( status.STATUS_NODE_NO_ERROR, status.STATUS_VREP_WAITING_4_CMD, "received write command")
	#DO WRITING WITH ROBOT HERE
	
	#self.send_status2gui( status.STATUS_NODE_NO_ERROR, status.STATUS_VREP_WAITING_4_CMD, "movement in progress")
	
	
        #rospy.logwarn('self.poly_y == 0')

        self.lock.release()
        
        
    def parse_input_from_gui(self, msg):
	cmd_list = msg.data.split("#", 1)        	
	
	#CONFIG
	self.config_list_string = (cmd_list[0].replace("config:", "", 1)).split(";")
	self.config_fontsize = int(self.config_list_string[0])
	self.config_pencil_length = float(self.config_list_string[1])	
	
	#DATA
	self.data_string = cmd_list[1].replace("data:", "", 1)
    
    def send_status2gui(self, nodestatus, vrepstatus, error_txt):
	self.status = nodestatus
	self.status_vrep = vrepstatus
	self.status_string = error_txt
	#set status: [nodestatus;vrepstatus;]
        commandStr = "status:" + str(self.status) + ";" + str(self.status_vrep)
        
        #set error text
        commandStr = commandStr + "#error:" + self.status_string        
        self.pub2gui.publish(commandStr)
        
        rospy.loginfo("send status 2 gui")
        #rospy.loginfo(commandStr)
        
        
        
if __name__ == '__main__':
    Node()
