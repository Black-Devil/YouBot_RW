# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os
import time

import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtCore import *#Qt, qWarning, Signal
from python_qt_binding.QtGui import *#QFileDialog, QIcon, QWidget

from youbot_rw_rqt_gui.msg import *
from std_msgs.msg import *
import vrep_controll
from std_msgs.msg import Empty

import status_intf as status



class YouBotGuiWidget(QWidget):
    """
    Widget for use with YouBot_RW
    """

    set_status_text = Signal(str)
    commandStr = ""
    

    def __init__(self, context):
        """ gui definitions
        @param [in] context <b><i><c> [string]: </c></i></b> plugin context hook to enable adding widgets as a ROS_GUI pane, ''PluginContext'
        """
        super(YouBotGuiWidget, self).__init__()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('youbot_rw_rqt_gui'), 'resource', 'YouBotGui_widget.ui')
        loadUi(ui_file, self)

        # init gui stuff
        self.processMode_comboBox.addItem("Writing")
        #self.processMode_comboBox.addItem("Draw Logo")
        self.processMode_comboBox.addItem("PTP Position")
        self.processMode_comboBox.addItem("PTP Angles")
        self.processMode_comboBox.addItem("LIN Position")
        #self.processMode_comboBox.addItem("LIN Angles")



        self.kinematic_comboBox.addItem("geometric")
        self.kinematic_comboBox.addItem("nummeric")

        self.processMode = status.PROCESSING_MODE_WRITING

        #self.my_node = rospy.init_node('youbot_rw_gui_node')
        self.pub_write_cmd = rospy.Publisher('/youbot_rw/gui2node', rw_node, tcp_nodelay=True,queue_size=1)
        self.pub_reset = rospy.Publisher('/youbot_rw/reset', Empty, tcp_nodelay=True,queue_size=1)
        rospy.Subscriber('/youbot_rw/node2gui', rw_node_state, self.callback_status_cmd)

        self.setObjectName('YouBot_RW_GUI')
        

        
        self.write_button.clicked[bool].connect(self._handle_write_clicked)
        self.resetButton.clicked[bool].connect(self._handle_reset_clicked)
        self.stopButton.clicked[bool].connect(self._handle_stop_clicked)
        self.set_status_text.connect(self._set_status_text)
       
        self.closeEvent = self.handle_close
        self.keyPressEvent = self.on_key_press
        # TODO when the closeEvent is properly called by ROS_GUI implement that event instead of destroyed
        #self.destroyed.connect(self.handle_destroy)
        
        #self.write_button.setEnabled(False)
        

    
    # callbacks for ui events
    def on_key_press(self, event):
        """ GUI Envent handler
        @param [in] event <b><i><c> [event]: </c></i></b> Event delivered from QT-GUI
        """

        key = event.key()
        #if key == Qt.Key_Space:
            
        #elif key == Qt.Key_Home:
            
        #elif key == Qt.Key_End:
            
        #elif key == Qt.Key_Plus or key == Qt.Key_Equal:
            
        #elif key == Qt.Key_Minus:
            
        #elif key == Qt.Key_Left:
            
        #elif key == Qt.Key_Right:
            
        #elif key == Qt.Key_Up or key == Qt.Key_PageUp:
            
        #elif key == Qt.Key_Down or key == Qt.Key_PageDown:
            

    #def handle_destroy(self, args):
        

    def handle_close(self, event):
        """ GUI Envent handler
        @param [in] event <b><i><c> [event]: </c></i></b> Event delivered from QT-GUI
        """
        event.accept()

    def _handle_stop_clicked(self):
        """ GUI Envent handler Stops V-REP
        """
        msg=Empty()
        self.pub_reset.publish(msg)
        vrep_controll.TriggerSimualtion()
        vrep_controll.stopSimualtion()
        vrep_controll.TriggerSimualtion()
        vrep_controll.TriggerSimualtion()

    def _handle_reset_clicked(self):
        """ GUI Envent handler, resets gui/node and V-REP
        """
        msg=Empty()
        self.pub_reset.publish(msg)
        vrep_controll.TriggerSimualtion()
        vrep_controll.stopSimualtion()
        vrep_controll.TriggerSimualtion()
        vrep_controll.startSimualtion()
        vrep_controll.TriggerSimualtion()
        vrep_controll.TriggerSimualtion()
        vrep_controll.startSimualtion()
        vrep_controll.TriggerSimualtion()
        vrep_controll.TriggerSimualtion()
        vrep_controll.startSimualtion()
        vrep_controll.TriggerSimualtion()


    def _handle_write_clicked(self):
        """ GUI Envent handler, sends chosen command to node
        """
        msg=rw_node()
        if(self.kinematic_comboBox.currentIndex() == 0):
            msg.kinematic = 0
        else:
            msg.kinematic = 1

        msg.res=self.res_spinBox.value()
        msg.Theta_1=self.theta_1_doubleSpinBox.value()
        msg.Theta_2=self.theta_2_doubleSpinBox.value()
        msg.Theta_3=self.theta_3_doubleSpinBox.value()
        msg.Theta_4=self.theta_4_doubleSpinBox.value()
        msg.Theta_5=self.theta_5_doubleSpinBox.value()
        msg.Pos_X=self.pos_x_doubleSpinBox.value()
        msg.Pos_Y=self.pos_y_doubleSpinBox.value()
        msg.Pos_Z=self.pos_z_doubleSpinBox.value()
        msg.letters=str(self.plainTextEdit_writeContent.toPlainText())
        msg.Fontsize=self.fontsize_spinBox.value()
        if(self.processMode_comboBox.currentIndex() == 0):
            msg.processmode = status.PROCESSING_MODE_WRITING
            self.processMode = status.PROCESSING_MODE_WRITING
        elif(self.processMode_comboBox.currentIndex() == 1):
            msg.processmode = status.PROCESSING_MODE_LOGO
            self.processMode = status.PROCESSING_MODE_LOGO
        elif(self.processMode_comboBox.currentIndex() == 2):
            msg.processmode = status.PROCESSING_MODE_PTP_POSITION
            self.processMode = status.PROCESSING_MODE_PTP_POSITION
        elif(self.processMode_comboBox.currentIndex() == 3):
            msg.processmode = status.PROCESSING_MODE_PTP_ANGLES
            self.processMode = status.PROCESSING_MODE_PTP_ANGLES
        elif(self.processMode_comboBox.currentIndex() == 4):
            msg.processmode = status.PROCESSING_MODE_LIN_POSITION
            self.processMode = status.PROCESSING_MODE_LIN_POSITION
        elif(self.processMode_comboBox.currentIndex() == 5):
            msg.processmode = status.PROCESSING_MODE_LIN_ANGLES
            self.processMode = status.PROCESSING_MODE_LIN_ANGLES

        self.pub_write_cmd.publish(msg)
        
        
    def callback_status_cmd(self,msg):
        """ Ros Subsciber callback, applys data from node to GUI
        @param [in] msg <b><i><c> [ros-message]: </c></i></b>
        """
	    #CONFIG
	    self.status_node_status = msg.nodestatus
	    self.status_vrep_status = msg.vrepstatus

	    #DATA
	    self.status_data_string = msg.error
	
	
	    if self.status_node_status == status.STATUS_NODE_NO_ERROR:
	        self.set_status_text.emit(self.status_data_string)

            if(self.processMode == status.PROCESSING_MODE_PTP_POSITION or self.processMode == status.PROCESSING_MODE_LIN_POSITION
                or self.processMode == status.PROCESSING_MODE_WRITING or self.processMode == status.PROCESSING_MODE_LOGO):
                # set angles
                self.theta_1_doubleSpinBox.setValue(msg.Theta_1)
                self.theta_2_doubleSpinBox.setValue(msg.Theta_2)
                self.theta_3_doubleSpinBox.setValue(msg.Theta_3)
                self.theta_4_doubleSpinBox.setValue(msg.Theta_4)
                self.theta_5_doubleSpinBox.setValue(msg.Theta_5)
            if(self.processMode == status.PROCESSING_MODE_PTP_ANGLES or self.processMode == status.PROCESSING_MODE_LIN_ANGLES
                or self.processMode == status.PROCESSING_MODE_WRITING or self.processMode == status.PROCESSING_MODE_LOGO):
                # set pos
                self.pos_x_doubleSpinBox.setValue(msg.Pos_X)
                self.pos_y_doubleSpinBox.setValue(msg.Pos_Y)
                self.pos_z_doubleSpinBox.setValue(msg.Pos_Z)


    def _set_status_text(self, text):
        """ Set's textlabel in gui
        @param [in] event <b><i><c> [string]: </c></i></b> String to show
        """
        if text:            
            self.status_label.setText(text)
        else:
            self.status_label.clear()

    

    #def shutdown_all(self):
        #self._timeline.handle_close()