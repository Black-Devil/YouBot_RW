import sys
import socket
from PyQt4 import QtGui, QtCore
import numpy as np

glob_ikSolVal = np.empty([8,6])

class GUI(QtGui.QWidget):
    def __init__(self):     
        super(GUI, self).__init__()
        
        # initialize GUI
        self.initUI()
        
        # get the initial axis values as well as the initial position and orientation of the robot
        self.dataTransfer('GET') # using the prefix get for server-sided parsing
                  
    
    def initUI(self):         
        grid = QtGui.QGridLayout()
        # create the group of widgets at the top left position
        grid.addWidget(self.createCurrAxesGroup(), 0, 0)
        # create the group of widgets at the top mid position
        grid.addWidget(self.createTrgtAxesGroup(), 0, 1)
        # create the group of widgets at the top right position
        grid.addWidget(self.createMoveGroup(), 0, 2)
        # create the group of widgets at the top right position
        grid.addWidget(self.createSettingGroup(), 0, 3)        
        # create the group of widgets at the bottom left position
        grid.addWidget(self.createCurrCartPosGroup(), 1, 0)
        # create the group of widgets at the bottom right position
        grid.addWidget(self.createTrgtCartPosGroup(), 1, 1, 1, 1)
        # create the group of widgets at the bottom right position
        grid.addWidget(self.createIkSolGroup(), 1, 2, 1, 2)

        
        # set window properties
        self.setLayout(grid)
        self.resize(600, 400)
        self.center()
        self.setWindowTitle('Simulation')
        self.show()
        
        
    # create the group of widgets at the top left position
    def createCurrAxesGroup(self):
        # initialize the group
        group_box = QtGui.QGroupBox('Current Axes')
        # use grid layout
        grid_axes = QtGui.QGridLayout()
        
        # set up labels
        label_axes_a1 = QtGui.QLabel('A1', self)    
        label_axes_a2 = QtGui.QLabel('A2', self)
        label_axes_a3 = QtGui.QLabel('A3', self)    
        label_axes_a4 = QtGui.QLabel('A4', self)    
        label_axes_a5 = QtGui.QLabel('A5', self)    
        label_axes_a6 = QtGui.QLabel('A6', self)
        # set up line edits
        self.lineedit_axes_a1 = QtGui.QLineEdit('0', self)
        self.lineedit_axes_a2 = QtGui.QLineEdit('0', self)
        self.lineedit_axes_a3 = QtGui.QLineEdit('0', self)   
        self.lineedit_axes_a4 = QtGui.QLineEdit('0', self)    
        self.lineedit_axes_a5 = QtGui.QLineEdit('0', self)    
        self.lineedit_axes_a6 = QtGui.QLineEdit('0', self)
        # disable line edits
        self.lineedit_axes_a1.setEnabled(0)
        self.lineedit_axes_a2.setEnabled(0)
        self.lineedit_axes_a3.setEnabled(0)
        self.lineedit_axes_a4.setEnabled(0)
        self.lineedit_axes_a5.setEnabled(0)
        self.lineedit_axes_a6.setEnabled(0)
        
        # add the widgets to the grid layout
        grid_axes.addWidget(label_axes_a1, 0, 0)
        grid_axes.addWidget(self.lineedit_axes_a1, 0, 1)
        grid_axes.addWidget(label_axes_a2, 1, 0)
        grid_axes.addWidget(self.lineedit_axes_a2, 1, 1)
        grid_axes.addWidget(label_axes_a3, 2, 0)
        grid_axes.addWidget(self.lineedit_axes_a3, 2, 1)
        grid_axes.addWidget(label_axes_a4, 3, 0)
        grid_axes.addWidget(self.lineedit_axes_a4, 3, 1)
        grid_axes.addWidget(label_axes_a5, 4, 0)
        grid_axes.addWidget(self.lineedit_axes_a5, 4, 1)
        grid_axes.addWidget(label_axes_a6, 5, 0)
        grid_axes.addWidget(self.lineedit_axes_a6, 5, 1)
        
        # set the grid layout for the group
        group_box.setLayout(grid_axes)
        
        return group_box
    
    
    def createMoveGroup(self):
        # initialize the group
        group_box = QtGui.QGroupBox('Movement')
        # use grid layout
        grid_mv = QtGui.QGridLayout()
        
        # set up radio buttons
        self.radio_asynch = QtGui.QRadioButton('asynchronous', self)
        self.radio_asynch.setChecked(True)
        
        """
        # set text color (dont function yet)
        palette_asynch = QtGui.QPalette()
        palette_asynch.setColor(QtGui.QPalette.Foreground, QtGui.QColor(255,0,0,255))
        self.radio_asynch.setPalette(palette_asynch)
        """
        
        self.radio_synch = QtGui.QRadioButton('synchronous', self)
        self.radio_lin = QtGui.QRadioButton('linear Movement', self)
        # set up button for the calculation
        self.button_move = QtGui.QPushButton('Move', self)
        # trigger function on clicked
        QtCore.QObject.connect(self.button_move, QtCore.SIGNAL("clicked()"), self.buttonMoveClicked)
        
        # add the widgets to the grid layout

        grid_mv.addWidget(self.radio_asynch, 0,0)     
        grid_mv.addWidget(self.radio_synch, 1,0)
        grid_mv.addWidget(self.radio_lin, 2,0)
        grid_mv.addWidget(self.button_move, 5,0)
        
        # set the grid layout for the group
        group_box.setLayout(grid_mv)
        
        return group_box
    
    def createSettingGroup(self):
        # initialize the group
        group_box = QtGui.QGroupBox('Settings')
        # use grid layout
        grid_set = QtGui.QGridLayout()
        
        # set up radio buttons
        self.check_drawPath = QtGui.QCheckBox('draw Path', self)
        self.check_drawPath.setChecked(True)
        self.check_autoclearPath = QtGui.QCheckBox('auto clear Path', self)
        self.check_autoclearPath.setChecked(True)
        self.check_ignoreSingularities = QtGui.QCheckBox('ignore Singularities', self)
        self.check_ignoreSingularities.setChecked(False)
        self.check_showPathSteps = QtGui.QCheckBox('show Path Step Points', self)
        self.check_showPathSteps.setChecked(False)
        self.check_showPathStepsHigh = QtGui.QCheckBox('show Path Step Highlighted', self)
        self.check_showPathStepsHigh.setChecked(True)
        self.check_showDebugCom = QtGui.QCheckBox('show server communication', self)
        self.check_showDebugCom.setChecked(False)
      
        # add the widgets to the grid layout

        grid_set.addWidget(self.check_drawPath, 0,0)     
        grid_set.addWidget(self.check_autoclearPath, 1,0)
        grid_set.addWidget(self.check_ignoreSingularities, 2,0)
        grid_set.addWidget(self.check_showPathSteps, 3,0)
        grid_set.addWidget(self.check_showPathStepsHigh, 4,0)
        grid_set.addWidget(self.check_showDebugCom, 5,0)
        
        # set the grid layout for the group
        group_box.setLayout(grid_set)
        
        return group_box
    
    
    # create the group of widgets at the bottom left position
    def createCurrCartPosGroup(self):
        # initialize the group
        group_box = QtGui.QGroupBox('Current Cartesian Position')
        # use grid layout
        grid_cartpos = QtGui.QGridLayout()
        
        # set up labels
        label_cartpos_x = QtGui.QLabel('X', self)
        label_cartpos_y = QtGui.QLabel('Y', self)
        label_cartpos_z = QtGui.QLabel('Z', self)
        label_cartpos_a = QtGui.QLabel('A', self)
        label_cartpos_b = QtGui.QLabel('B', self)
        label_cartpos_c = QtGui.QLabel('C', self)
        # set up line edits 
        self.lineedit_cartpos_x = QtGui.QLineEdit('0', self)
        self.lineedit_cartpos_y = QtGui.QLineEdit('0', self)  
        self.lineedit_cartpos_z = QtGui.QLineEdit('0', self)
        self.lineedit_cartpos_a = QtGui.QLineEdit('0', self) 
        self.lineedit_cartpos_b = QtGui.QLineEdit('0', self) 
        self.lineedit_cartpos_c = QtGui.QLineEdit('0', self)
        # disable line edits
        self.lineedit_cartpos_x.setEnabled(0)
        self.lineedit_cartpos_y.setEnabled(0)
        self.lineedit_cartpos_z.setEnabled(0)
        self.lineedit_cartpos_a.setEnabled(0)
        self.lineedit_cartpos_b.setEnabled(0)
        self.lineedit_cartpos_c.setEnabled(0)
        
        # add the widgets to the grid layout
        grid_cartpos.addWidget(label_cartpos_x, 0, 0)
        grid_cartpos.addWidget(self.lineedit_cartpos_x, 0, 1)
        grid_cartpos.addWidget(label_cartpos_y, 1, 0)
        grid_cartpos.addWidget(self.lineedit_cartpos_y, 1, 1)
        grid_cartpos.addWidget(label_cartpos_z, 2, 0)
        grid_cartpos.addWidget(self.lineedit_cartpos_z, 2, 1)
        grid_cartpos.addWidget(label_cartpos_a, 3, 0)
        grid_cartpos.addWidget(self.lineedit_cartpos_a, 3, 1)
        grid_cartpos.addWidget(label_cartpos_b, 4, 0)
        grid_cartpos.addWidget(self.lineedit_cartpos_b, 4, 1)
        grid_cartpos.addWidget(label_cartpos_c, 5, 0)
        grid_cartpos.addWidget(self.lineedit_cartpos_c, 5, 1)
        
        # set the grid layout for the group
        group_box.setLayout(grid_cartpos)
        
        return group_box
        
    
    # create the group of widgets at the top right position
    def createTrgtAxesGroup(self):
        # initialize the group
        group_box = QtGui.QGroupBox('Target Axes')
        # use grid layout
        grid_ptp = QtGui.QGridLayout()
        
        # set up labels
        label_ptp_a1 = QtGui.QLabel('A1', self)
        label_ptp_a2 = QtGui.QLabel('A2', self)
        label_ptp_a3 = QtGui.QLabel('A3', self)
        label_ptp_a4 = QtGui.QLabel('A4', self)
        label_ptp_a5 = QtGui.QLabel('A5', self)
        label_ptp_a6 = QtGui.QLabel('A6', self)
        # set up line edits 
        self.lineedit_ptp_a1 = QtGui.QLineEdit('0', self)  
        self.lineedit_ptp_a2 = QtGui.QLineEdit('0', self)  
        self.lineedit_ptp_a3 = QtGui.QLineEdit('0', self) 
        self.lineedit_ptp_a4 = QtGui.QLineEdit('0', self) 
        self.lineedit_ptp_a5 = QtGui.QLineEdit('0', self) 
        self.lineedit_ptp_a6 = QtGui.QLineEdit('0', self)   
        
        # add the widgets to the grid layout
        grid_ptp.addWidget(label_ptp_a1, 0, 0)
        grid_ptp.addWidget(self.lineedit_ptp_a1, 0, 1)
        grid_ptp.addWidget(label_ptp_a2, 1, 0)
        grid_ptp.addWidget(self.lineedit_ptp_a2, 1, 1)
        grid_ptp.addWidget(label_ptp_a3, 2, 0)
        grid_ptp.addWidget(self.lineedit_ptp_a3, 2, 1)
        grid_ptp.addWidget(label_ptp_a4, 3, 0)
        grid_ptp.addWidget(self.lineedit_ptp_a4, 3, 1)
        grid_ptp.addWidget(label_ptp_a5, 4, 0)
        grid_ptp.addWidget(self.lineedit_ptp_a5, 4, 1)
        grid_ptp.addWidget(label_ptp_a6, 5, 0)
        grid_ptp.addWidget(self.lineedit_ptp_a6, 5, 1)
        
        # set the grid layout for the group
        group_box.setLayout(grid_ptp)
        
        return group_box
        
        
    # create the group of widgets at the bottom right position
    def createTrgtCartPosGroup(self):
        # initialize the group
        group_box = QtGui.QGroupBox('Target Cartesian Position')
        # use grid layout
        grid_cartptp = QtGui.QGridLayout()
        
        # set up labels
        label_cartptp_x = QtGui.QLabel('X', self)
        label_cartptp_y = QtGui.QLabel('Y', self)
        label_cartptp_z = QtGui.QLabel('Z', self)
        label_cartptp_a = QtGui.QLabel('A', self)
        label_cartptp_b = QtGui.QLabel('B', self)
        label_cartptp_c = QtGui.QLabel('C', self)  
        # set up line edits 
        self.lineedit_cartptp_x = QtGui.QLineEdit('0', self)
        self.lineedit_cartptp_y = QtGui.QLineEdit('0', self)  
        self.lineedit_cartptp_z = QtGui.QLineEdit('0', self)
        self.lineedit_cartptp_a = QtGui.QLineEdit('0', self) 
        self.lineedit_cartptp_b = QtGui.QLineEdit('0', self) 
        self.lineedit_cartptp_c = QtGui.QLineEdit('0', self)
        
        # add the widgets to the grid layout
        grid_cartptp.addWidget(label_cartptp_x, 0, 0)
        grid_cartptp.addWidget(self.lineedit_cartptp_x, 0, 1)
        grid_cartptp.addWidget(label_cartptp_y, 1, 0)
        grid_cartptp.addWidget(self.lineedit_cartptp_y, 1, 1)
        grid_cartptp.addWidget(label_cartptp_z, 2, 0)
        grid_cartptp.addWidget(self.lineedit_cartptp_z, 2, 1)
        grid_cartptp.addWidget(label_cartptp_a, 3, 0)
        grid_cartptp.addWidget(self.lineedit_cartptp_a, 3, 1)
        grid_cartptp.addWidget(label_cartptp_b, 4, 0)
        grid_cartptp.addWidget(self.lineedit_cartptp_b, 4, 1)
        grid_cartptp.addWidget(label_cartptp_c, 5, 0)
        grid_cartptp.addWidget(self.lineedit_cartptp_c, 5, 1)
        
        # set the grid layout for the group
        group_box.setLayout(grid_cartptp)
        
        return group_box


    # create the group of widgets at the bottom right position
    def createIkSolGroup(self):
        # initialize the group
        group_box = QtGui.QGroupBox('Inverse Kinematics Solutions')
        # use grid layout
        grid_iksol = QtGui.QGridLayout()
        
        
        # set up the line edit box for the multiple kinematic solutions
        #self.lineedit_cartptp_box = QtGui.QTextEdit(self)
        #self.lineedit_cartptp_box.setSizePolicy(QtGui.QSizePolicy.Expanding,QtGui.QSizePolicy.Expanding)
        self.combo_ikSolutions = QtGui.QComboBox(self)
        self.combo_ikSolutions.setSizePolicy(QtGui.QSizePolicy.Expanding,QtGui.QSizePolicy.Fixed)
        self.combo_ikSolutions.setEditable(False)
        # set up button for the calculation
        self.button_calculate = QtGui.QPushButton('Calculate IK', self)
        # trigger function on clicked
        QtCore.QObject.connect(self.button_calculate, QtCore.SIGNAL("clicked()"), self.buttonCalculateClicked)
        QtCore.QObject.connect(self.combo_ikSolutions, QtCore.SIGNAL("activated(int)"), self.comboActivated)
        
        # add the widgets to the grid layout
        grid_iksol.addWidget(self.button_calculate, 5, 0)
        grid_iksol.addWidget(self.combo_ikSolutions, 0, 0, 1, 1)
        #grid_iksol.addWidget(self.lineedit_cartptp_box, 2, 0, 2, 1)
        # set the grid layout for the group
        group_box.setLayout(grid_iksol)
        
        return group_box

    # setting window properties
    def center(self):
        qr = self.frameGeometry()
        cp = QtGui.QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())
        
        
    # function is called when the move button is clicked
    def buttonMoveClicked(self):
        # prefix for parsing
        prefix = "MOV#"   
        # get values and convert QString to string
        msg = str(self.lineedit_ptp_a1.text()+";"+self.lineedit_ptp_a2.text()+";"+self.lineedit_ptp_a3.text()+";"+self.lineedit_ptp_a4.text()+";"+self.lineedit_ptp_a5.text()+";"+self.lineedit_ptp_a6.text()) 
        # get motion type
        if self.radio_asynch.isChecked() is True:
            motion_type = "#A"
        elif self.radio_synch.isChecked() is True:
            motion_type = "#S"
        elif self.radio_lin.isChecked() is True:
            motion_type = "#L"
        
        # get draw path setting
        if self.check_drawPath.isChecked() is True:
            drawPath = "#T"
        else:
            drawPath = "#F"
        
        #get auto clear path setting    
        if self.check_autoclearPath.isChecked() is True:
            autoClearPath = "#T"
        else:
            autoClearPath = "#F"
            
            
        #get ignore singularities setting    
        if self.check_ignoreSingularities.isChecked() is True:
            ignoreSingularties = "#T"
        else:
            ignoreSingularties = "#F"
        
        #get show path steps setting    
        if self.check_showPathSteps.isChecked() is True:
            showPathSteps = "#T"
        else:
            showPathSteps = "#F"
        
        #get show path steps highlighted setting    
        if self.check_showPathStepsHigh.isChecked() is True:
            showPathStepsHigh = "#T"
        else:
            showPathStepsHigh = "#F"
        
        #get show server com setting    
        if self.check_showDebugCom.isChecked() is True:
            showSrvrCom = "#T"
        else:
            showSrvrCom = "#F"


        # send data
        self.dataTransfer(prefix+msg+motion_type+drawPath+autoClearPath+ignoreSingularties+showPathSteps+showPathStepsHigh+showSrvrCom)
        
    # function is called when the calculate IK button is clicked
    def buttonCalculateClicked(self):
        # prefix for parsing
        prefix = "CAL#"   
        # get values and convert QString to string
        values = str(self.lineedit_cartptp_x.text()+";"+self.lineedit_cartptp_y.text()+";"+self.lineedit_cartptp_z.text()+";"+self.lineedit_cartptp_a.text()+";"+self.lineedit_cartptp_b.text()+";"+self.lineedit_cartptp_c.text()) 
        # send data
        
        #get show server com setting    
        if self.check_showDebugCom.isChecked() is True:
            showSrvrCom = "#T"
        else:
            showSrvrCom = "#F"
            
        self.dataTransfer(prefix+values+showSrvrCom)
    
        
    def comboActivated(self, curr_i):
        
        #print "Activated"
        #print curr_i
        
        global glob_ikSolVal
        
        if curr_i >= 0:            
            ikSol = glob_ikSolVal[curr_i] 
            self.lineedit_ptp_a1.setText(str(ikSol[0]))
            self.lineedit_ptp_a2.setText(str(ikSol[1]))
            self.lineedit_ptp_a3.setText(str(ikSol[2]))
            self.lineedit_ptp_a4.setText(str(ikSol[3]))
            self.lineedit_ptp_a5.setText(str(ikSol[4]))
            self.lineedit_ptp_a6.setText(str(ikSol[5]))
        

    # handles the data transfer between the GUI (client) and openrave (server)
    def dataTransfer(self, msg):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM, socket.IPPROTO_TCP)
        s.connect(('localhost', 54321))
        s.send(msg)
        recv_data, addr = s.recvfrom(2048)
        
        self.handleData(recv_data)
        
        #test server com (debug)
        #print recv_data
        
        s.close()
      
    
    # handles the data received from openrave  
    def handleData(self, data):
        data_arr = data.split("#")
        
        #print data for debug
        #print data
        
        if data_arr[0] == "VAL":
            self.updateValues(data_arr)
        elif data_arr[0] == "INK":
            self.updateINK(data_arr)
            
            
            
    # update UI with received position, orientation and axis values
    def updateValues(self, data):
        # update axis values
        axis_arr = data[1].split(";")
        self.lineedit_axes_a1.setText(axis_arr[0])
        self.lineedit_axes_a2.setText(axis_arr[1])
        self.lineedit_axes_a3.setText(axis_arr[2])
        self.lineedit_axes_a4.setText(axis_arr[3])
        self.lineedit_axes_a5.setText(axis_arr[4])
        self.lineedit_axes_a6.setText(axis_arr[5])
        
        # update position and orientaion values
        cart_arr = data[2].split(";")
        self.lineedit_cartpos_x.setText(cart_arr[0])
        self.lineedit_cartpos_y.setText(cart_arr[1])
        self.lineedit_cartpos_z.setText(cart_arr[2])
        self.lineedit_cartpos_a.setText(cart_arr[3])
        self.lineedit_cartpos_b.setText(cart_arr[4])
        self.lineedit_cartpos_c.setText(cart_arr[5])
        
    
    # update UI with multiple inverse kinematic solutions
    def updateINK(self, data):

        global glob_ikSolVal
                
        self.combo_ikSolutions.clear()    
        
        #self.lineedit_cartptp_box.setText(data[1])
        # get string with values
        inkSolStr = data[1].split('$')
        
        if not inkSolStr[0] =="":                                    
            inkSolVal = np.empty([len(inkSolStr), 6])
            
            #generate ink solutions list
            for i in range(len(inkSolStr)):
                #get solution string
                tmp_inkSolVal = inkSolStr[i].split(";")
                #parse string to float solution
                inkSolVal[i,0] = float(tmp_inkSolVal[0])
                inkSolVal[i,1] = float(tmp_inkSolVal[1])
                inkSolVal[i,2] = float(tmp_inkSolVal[2])
                inkSolVal[i,3] = float(tmp_inkSolVal[3])
                inkSolVal[i,4] = float(tmp_inkSolVal[4])
                inkSolVal[i,5] = float(tmp_inkSolVal[5])
                
                #put solutions in the combo box
                self.combo_ikSolutions.addItem(inkSolStr[i])
                                   
            glob_ikSolVal = inkSolVal
    
def main():
    app = QtGui.QApplication(sys.argv)
    
    gui = GUI()
    
    sys.exit(app.exec_())
    
if __name__ == '__main__':
    main()