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

from std_msgs.msg import *



class YouBotGuiWidget(QWidget):
    """
    Widget for use with YouBot_RW
    """

    set_status_text = Signal(str)
    commandStr = ""
    

    def __init__(self, context):
        """
        :param context: plugin context hook to enable adding widgets as a ROS_GUI pane, ''PluginContext''
        """
        super(YouBotGuiWidget, self).__init__()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('youbot_rw_rqt_gui'), 'resource', 'YouBotGui_widget.ui')
        loadUi(ui_file, self)
        
        #self.my_node = rospy.init_node('youbot_rw_gui_node')
        self.pub_write_cmd = rospy.Publisher('/youbot_rw/write_cmd', String, tcp_nodelay=True,queue_size=1)

        self.setObjectName('YouBot_RW_GUI')
        

        
        self.write_button.clicked[bool].connect(self._handle_write_clicked)
        self.set_status_text.connect(self._set_status_text)
       
        self.closeEvent = self.handle_close
        self.keyPressEvent = self.on_key_press
        # TODO when the closeEvent is properly called by ROS_GUI implement that event instead of destroyed
        #self.destroyed.connect(self.handle_destroy)
        
        #self.write_button.setEnabled(False)
        

    
    # callbacks for ui events
    def on_key_press(self, event):
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
        #self.shutdown_all()

        event.accept()   


    def _handle_write_clicked(self):
	#status
	self.set_status_text.emit("send write command")        
        
        #set config: [Fontsize;PencilLength;]
        commandStr = "config:" + str(self.fontsize_spinBox.value()) + ";" + str(self.pencilLength_doubleSpinBox.value())
        
        #set text data
        commandStr = commandStr + "#data:" + str(self.plainTextEdit_writeContent.toPlainText())
        self.pub_write_cmd.publish(commandStr)    
    

    def _set_status_text(self, text):
        if text:            
            self.status_label.setText(text)
        else:
            self.status_label.clear()

    

    #def shutdown_all(self):
        #self._timeline.handle_close()