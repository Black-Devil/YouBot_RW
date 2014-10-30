#!/usr/bin/env python

from time import sleep, time
#import math
import threading

import rospy
from std_msgs.msg import *

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
        
        rospy.Subscriber('/youbot_rw/write_cmd', String, self.callback_write_cmd)
        self.pub_output = rospy.Publisher('/youbot_rw/output', String, tcp_nodelay=True,queue_size=1)
        
        self.spin_restarter()
        
    def init_params(self):
        self.rate = 100
        self.disable = False
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
        #msg.data
        
        cmd_list = msg.data.split("#", 1)        	
	
	rospy.loginfo(cmd_list[0])
	rospy.loginfo(cmd_list[1])
	self.pub_output.publish(msg.data)
	
        #rospy.logwarn('self.poly_y == 0')

        self.lock.release()
        
        
        
        
if __name__ == '__main__':
    Node()
