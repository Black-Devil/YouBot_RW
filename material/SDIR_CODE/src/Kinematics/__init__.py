from openravepy import *
import openravepy as op
#import openravepy.Robot as ro
import KinematicFunctions as kf
import MotionFunctions as mf
#from MotionFunctions import *
import numpy as np
import sys
import socket

#global handle for coordinate system drawing
glob_cs_handles = []
glob_TCP_handles = []
glob_cs_color = np.array([0,0,0])
glob_print_com = False

# handles the data transfer between openrave (server) and the GUI (client)
def dataTransfer():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM, socket.IPPROTO_TCP)
    
    s.bind(('localhost', 54321))
    s.listen(1)
    
    try:
        while True:
            komm, addr = s.accept()     
            while True:
                data = komm.recv(2048)
                if not data:
                    komm.close()
                    break
                
                global glob_print_com
                
                if glob_print_com is True:
                    print '|GUI Input |: ' + data   
                              
                # parse received data and get the string that should be sent back to the GUI
                send = handleData(data)
                if glob_print_com is True:
                    print '|GUI Output|: ' + send 
                
                # send new informations to the GUI for updating purposes
                komm.sendto(send, addr)
    finally:
        s.shutdown(socket.SHUT_RDWR)
        s.close()



# handles the data received from the GUI and sets up data for sending
def handleData(data):
    # split data string
    data_arr = data.split("#")
    
    global glob_cs_handles
    global glob_TCP_handles
    global glob_print_com
    
    # check if GUI requests the current robot axis values as well as current orientation and position 
    if data_arr[0] == 'GET':
        # prefix for parsing
        prefix = "VAL#"
        # get Axis values (OpenRaveAngles)
        axis_dof_arr = robot.GetDOFValues()
        
        #subtract offset (OpenRaveAngles-->ourAngles)
        axis_dof_arr = kf.SubstractOffsetToAngles(axis_dof_arr)
        
        #round gui output
        #TODO: round till non zero values or max digit
        axis_arr = np.around(axis_dof_arr, 2)
    
        # convert to string
        axis_values = str(axis_arr[0])+";"+str(axis_arr[1])+";"+str(axis_arr[2])+";"+str(axis_arr[3])+";"+str(axis_arr[4])+";"+str(axis_arr[5])+'#'

        #center of coordinate system
        center_of_cs = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

        #calculate the transformation from coordinate system 0 to 7 with center of 7 as point in 7     
        dk_point_mat = kf.CalculateDirectKinematicsTransformation(axis_dof_arr, center_of_cs, 0, 7)
        
        #use global handle for drawing cs
        glob_cs_handles = []
        
        #initialize orientation array
        orientation = np.array([0.0,0.0,0.0])
        
        #generate orientation from transformation matrix
        orientation = kf.GetOrientationFromTransformation(dk_point_mat)
        
        #fill point array with position and orientation of point         
        point_arr = np.around(np.array([dk_point_mat[0,3],
                                        dk_point_mat[1,3],
                                        dk_point_mat[2,3],
                                        orientation[0],
                                        orientation[1],
                                        orientation[2],
                                        dk_point_mat[3,3]]),4)
        
        #draw cs
        glob_cs_handles.append(misc.DrawAxes(env,dk_point_mat,0.1,3,glob_cs_color))
        
        cart_values = str(point_arr[0])+";"+str(point_arr[1])+";"+str(point_arr[2])+";"+str(point_arr[3])+";"+str(point_arr[4])+";"+str(point_arr[5])     
        

        return prefix+axis_values+cart_values
    
    # check if the robot should be moved 
    elif data_arr[0] == 'MOV':
        # get values
        values = data_arr[1].split(';')
        # convert from string to float and save in numpy array
        # add offset to angles (ourAngles-->OpenRaveAngles)
        target = kf.AddOffsetToAngles(values)
                
        # get the motion type
        motion_type = data_arr[2]
        
        # get the drawPath setting
        if data_arr[3] == "T":
            drawPath = True
        else:
            drawPath = False
        
        # get the auto clear path setting
        if data_arr[4] == "T":
            autoClearPath = True
        else:
            autoClearPath = False
            
        # get the ignore singularities setting
        if data_arr[5] == "T":
            ignoreSingularities = True
        else:
            ignoreSingularities = False
            
        # get the show path step point setting
        if data_arr[6] == "T":
            showPathSteps = True
        else:
            showPathSteps = False
            
        # get the show path steps highlighted setting
        if data_arr[7] == "T":
            showPathStepsHigh = True
        else:
            showPathStepsHigh = False
            
        # get the server com setting
        if data_arr[8] == "T":
            showSrvrCom = True
        else:
            showSrvrCom = False
        glob_print_com = showSrvrCom
            
        # check if target is in range
        
        temp_ourAngles = kf.SubstractOffsetToAngles(target)
        reachableAngles = kf.CheckAngleConstraints(temp_ourAngles)
        reachablePos = False
        
        if reachableAngles is True:   
            # get trajectory
            if data_arr[2] == 'L':
                trajectory = mf.LINtoConfiguration(robot.GetDOFValues(), target, 0.5, 1.0, robot, ignoreSingularities)
            else:
                trajectory = mf.PTPtoConfiguration(robot.GetDOFValues(), target, motion_type)
            
            # calculate trajectory points
            traj_points = mf.calcTrajPoints(trajectory)
            
            reachablePos = kf.CheckPosConstraints(traj_points[traj_points.shape[0]-1])
            if reachablePos is False:
                print "|WARNING|: Position constraints were hurt! Target is not reachable! Movement will not be started."
             
        else:
            print "|WARNING|: Angle constraints were hurt! Target is not reachable! Movement will not be started." 
            

        # move robot
        if reachableAngles is True and reachablePos is True:
            mf.Move(env, robot, trajectory, traj_points, motion_type, drawPath, autoClearPath, showPathSteps, showPathStepsHigh)
                
        # send new information about the robot's axis values, position and orientation to the GUI for updating purpose
        # prefix for parsing
        prefix = "VAL#"
        # get Axis values
        axis_dof_arr = robot.GetDOFValues()

        #subtract offset (OpenRaveAngles-->ourAngles)
        axis_dof_arr = kf.SubstractOffsetToAngles(axis_dof_arr)
        
        #center of coordinate system with orientation as a transformation matrix
        center_of_cs = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
        
        #calculate the transformation from coordinate system 0 to 7 with center of 7 as point in 7
        #TODO: put in ourAngles for calculation    
        dk_point_mat = kf.CalculateDirectKinematicsTransformation(axis_dof_arr, center_of_cs, 0, 7)
        
        #round gui output
        #TODO: round till non zero values or max digit
        axis_arr = np.around(axis_dof_arr, 4)
        
        # convert to string
        axis_values = str(axis_arr[0])+";"+str(axis_arr[1])+";"+str(axis_arr[2])+";"+str(axis_arr[3])+";"+str(axis_arr[4])+";"+str(axis_arr[5])+'#'
        
        #use global handle for drawing cs
        glob_cs_handles = []
        
        #initialize orientation array
        orientation = np.array([0.0,0.0,0.0])
        
        #generate orientation from transformation matrix
        orientation = kf.GetOrientationFromTransformation(dk_point_mat)

        #fill point array with position and orientation of point         
        point_arr = np.around(np.array([dk_point_mat[0,3],
                                        dk_point_mat[1,3],
                                        dk_point_mat[2,3],
                                        orientation[0],
                                        orientation[1],
                                        orientation[2],
                                        dk_point_mat[3,3]]),4)
        
        #draw cs
        glob_cs_handles.append(misc.DrawAxes(env,dk_point_mat,0.1,3,glob_cs_color))
        
        cart_values = str(point_arr[0])+";"+str(point_arr[1])+";"+str(point_arr[2])+";"+str(point_arr[3])+";"+str(point_arr[4])+";"+str(point_arr[5])     
        return prefix+axis_values+cart_values
    
    # check if inverse kinematics should be calculated
    if data_arr[0] == "CAL":
        # get string with values
        values = data_arr[1].split(';')
        
        # get the server com setting
        if data_arr[2] == "T":
            showSrvrCom = True
        else:
            showSrvrCom = False
        glob_print_com = showSrvrCom
        
        #generate point array
        point = np.array([float(values[0]), 
                          float(values[1]), 
                          float(values[2]), 
                          float(values[3]), 
                          float(values[4]), 
                          float(values[5])])
        
        #initialize matrix for possible angles
        target = np.empty([8,6])
        
        #initialize transformation matrix
        cs_mat = np.empty([4,4])
        
        #fill transformation matrix
        cs_mat = kf.GetTransformationFromPoint(point)
        
        #draw TCP cs
        glob_TCP_handles = []
        
        glob_TCP_handles.append(misc.DrawAxes(env,cs_mat,0.1,3,glob_cs_color))
        
        #calculate inverse kinematic solutions
        target = kf.CalculateInverseKinematics(cs_mat)
        
        # send the (multiple) solutions to the GUI
        # prefix for parsing

        # adding dummy values (you need to replace them with the solutions)
        
        ik_values = ""
        firstSolIn = False
        
        for i in range(0,8):
            one_target = target[i]   
            one_target = np.around(one_target, 8)
        
            #check solution
            if kf.CheckAngleConstraints(one_target):
                if firstSolIn:
                    ik_values = ik_values+'$'
                ik_values = ik_values+ str(one_target[0])+";"+str(one_target[1])+";"+str(one_target[2])+";"+str(one_target[3])+";"+str(one_target[4])+";"+str(one_target[5])
                firstSolIn = True                
        
        prefix = "INK#"
        
        return prefix+ik_values
    
    
if __name__ == "__main__":
    # setting up the operave environment
    env = op.Environment() # create openrave environment
    env.SetViewer('qtcoin') # attach viewer (optional)
    env.Load('../../MyData/MyEnvironment/MyEnv.xml') # load a simple scene
    handles = []
    robot = env.GetRobots()[0] # get the first robot
    dataTransfer()
    
    
