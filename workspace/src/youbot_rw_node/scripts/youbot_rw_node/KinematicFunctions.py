#!/usr/bin/env python
import numpy as np

#DH Parameter Matrix
dhParameter = np.mat([[0.0,     0.0,       0.0,         0          ],
                      [0.147,   0.033,     0.0,         np.pi/2    ],
                      [0.0,     0.155,     0.0,         0.0        ],
                      [0.0,     0.135,     0.0,         0.0        ],
                      [0.0,     0.0,       np.pi/2,     np.pi/2    ],
                      [0.2175,  0.0,       0.0,         0   ]])

#distance value for computation of angle two (shoulder)
w = np.sqrt(np.power(0.145,2)+np.power(1.545,2))

#angle offset for computation of angle three (elbow)
theta_offset = np.arctan2(0.145,1.545)

#angle constraints
joint_one_low = np.deg2rad(-185)
joint_one_high = np.deg2rad(185)
joint_two_low = np.deg2rad(-135)
joint_two_high = np.deg2rad(35)
joint_three_low = np.deg2rad(-120)
joint_three_high = np.deg2rad(158)
joint_four_low = np.deg2rad(-350)
joint_four_high = np.deg2rad(350)
joint_five_low = np.deg2rad(-130)
joint_five_high = np.deg2rad(130)
joint_six_low = np.deg2rad(-350)
joint_six_high = np.deg2rad(350)


def AddOffsetToAngles(arr_angles):
    """add offset to angles (ourAngles-->OpenRaveAngles)
    
    :param arr_angles: array of angles for the joints
    :type array of strings
    :returns: array of angles for the joints with offset in it
    :rtype: array of floats
    """
    new_arr_angles = np.array([float(arr_angles[0])-np.pi/2, 
                           -1* float(arr_angles[1])-np.pi/2, 
                           -1* float(arr_angles[2])+np.pi/2, 
                           float(arr_angles[3]), 
                           -1* float(arr_angles[4]), 
                           float(arr_angles[5])])    
    
    return new_arr_angles

def SubstractOffsetToAngles(arr_angles):
    """substract offset from angles (OpenRaveAngles-->ourAngles)
    
    :param arr_angles: array of angles for the joints with offset
    :type array of floats
    :returns: array of angles for the joints without offset in it
    :rtype: array of floats
    """
    new_arr_angles = np.array([float(arr_angles[0])+np.pi/2, 
                           -1* (float(arr_angles[1])+np.pi/2), 
                           -1* (float(arr_angles[2])-np.pi/2), 
                           float(arr_angles[3]), 
                           -1* float(arr_angles[4]), 
                           float(arr_angles[5])])    
    
    return new_arr_angles

def CalculateDirectKinematicsTransformation(dofValues, point, origin, destination):

    """Caculate the transformation for direct kinematics
    
    :param point: transformation matrix for point (TCP) or vector of point
    :type matrix of floats or array of floats
    :param origin: the first coordinate system (smaller number)
    :type integer
    :param destination: the last coordinate system (the TCP coordinate system lies here) (higher number)
    :type integer
    :returns: transformation matrix from origin to point
    :rtype: matrix of floats
    """

    joint = 0
    angle = 0
        
    #calculate transformation from origin to destination
    for i in range(origin, destination):
        
        #for direct kinematics last joint first
        joint = destination - i - 1 + origin
        
        #first transformation theta is always zero
        if joint == 0:
            angle = 0
        else:
            angle = dofValues[joint - 1]
            
        #get dh parameters and use dh transformation matrix
        nextMatrix = GetTransformationForDh(GetDhForJoint(angle, joint))
        
        #matrix multiplication
        point = np.mat(nextMatrix) * np.mat(point)

    return point

def CalculateInverseKinematicsTransformation(point, angle, origin, destination):

    """Caculate the transformation for inverse kinematics
    
    :param point: transformation matrix for point (TCP) or vector of point
    :type matrix of floats or array of floats
    :param angle: the theta angle
    :type float
    :param origin: the first coordinate system (higher number)
    :type integer
    :param destination: the last coordinate system (the TCP coordinate system lies here) (smaller number)
    :type integer
    :returns: transformation matrix from origin to point
    :rtype: matrix of floats
    """

    #calculate inverse transformation from destination to origin
    for i in range(destination, origin):
        
        #get dh parameters
        dh_buffer = GetDhForJoint(angle, i)
        
        #use dh transformation matrix
        nextMatrix = GetInverseTransformationForDh(dh_buffer)
        
        #matrix multiplication
        point = np.mat(nextMatrix) * np.mat(point)

    return point

def GetTransformationForDh(params):
    
    """Calculate direct transformation matrix for one set of dh parameter
    
    :param params: the dh parameter for one transformation
    :type array of floats
    :returns: transformation matrix with dh parameters
    :rtype: matrix of floats
    """    
    
    transformation = np.empty([4, 4])
    
    #direct dh transformation matrix
    
    transformation[0][0] = np.cos(params[2])
    transformation[0][1] = -1*np.sin(params[2])*np.cos(params[3])
    transformation[0][2] = np.sin(params[2])*np.sin(params[3])
    transformation[0][3] = params[0]*np.cos(params[2])
    
    transformation[1][0] = np.sin(params[2])
    transformation[1][1] = np.cos(params[2])*np.cos(params[3])
    transformation[1][2] = -1*np.cos(params[2])*np.sin(params[3])
    transformation[1][3] = params[0]*np.sin(params[2])
    
    transformation[2][0] = 0
    transformation[2][1] = np.sin(params[3])
    transformation[2][2] = np.cos(params[3])
    transformation[2][3] = params[1]
    
    transformation[3][0] = 0
    transformation[3][1] = 0
    transformation[3][2] = 0
    transformation[3][3] = 1
    
    return transformation

def GetInverseTransformationForDh(params):
    
    """Calculate inverse transformation matrix for one set of dh parameter
    
    :param params: the dh parameter for one transformation
    :type array of floats
    :returns: inverse transformation matrix with dh parameters
    :rtype: matrix of floats
    """    
    
    transformation = np.empty([4, 4])
    
    #inverse dh transformation matrix
    
    transformation[0][0] = np.cos(params[2])
    transformation[0][1] = np.sin(params[2])
    transformation[0][2] = 0
    transformation[0][3] = -1*params[0]
    
    transformation[1][0] = -1*np.sin(params[2])*np.cos(params[3])
    transformation[1][1] = np.cos(params[2])*np.cos(params[3])
    transformation[1][2] = np.sin(params[3])
    transformation[1][3] = -1*params[1]*np.sin(params[3])
    
    transformation[2][0] = np.sin(params[2])*np.sin(params[3])
    transformation[2][1] = -1*np.cos(params[2])*np.sin(params[3])
    transformation[2][2] = np.cos(params[3])
    transformation[2][3] = -1*params[1]*np.cos(params[3])
    
    transformation[3][0] = 0
    transformation[3][1] = 0
    transformation[3][2] = 0
    transformation[3][3] = 1
    
    return transformation

def GetDhForJoint(angle, joint):

    """build dh parameters for one joint
    
    :param joint: the joint which represents one transformation
    :type integer
    :returns: the dh parameter for one transformation of joint
    :rtype: array of floats
    """    

    dhArray = np.empty([4])
    
    #get dh parameters from dh table
    
    dhArray[2] = angle

    dhArray[0] = dhParameter[joint,0]
    dhArray[1] = dhParameter[joint,1]
    dhArray[2] += dhParameter[joint,2] 
    dhArray[3] = dhParameter[joint,3]
 
    return dhArray

def GetOrientationFromTransformation(transformation):

    """get orientation angles from transformation matrix
    
    :param transformation: the transformation matrix for a cs
    :type matrix of floats
    :returns: the three orientation angles
    :rtype: array of floats
    """    
    
    x = -1*transformation[2,0]
    
    y = np.power(transformation[0,0],2)+np.power(transformation[1,0],2)
    
    y = np.sqrt(y)
    
    beta = 0
    
    if not(x == 0 and y == 0):
        beta = np.arctan2(x,y)
    
    alpha = 0
    
    if not(beta == np.pi/2 or beta == -np.pi/2):
        divide = np.cos(beta)
            
        x = transformation[1,0] / divide
        y = transformation[0,0] / divide
        alpha = np.arctan2(x,y)

    gamma = 0

    if not(beta == np.pi/2 or beta == -np.pi/2):
        divide = np.cos(beta)
        
        x = transformation[2,1] / divide
        y = transformation[2,2] / divide
        gamma = np.arctan2(x,y)
    else:
        x = transformation[0,1]
        y = transformation[1,1]
        gamma = np.arctan2(x,y) * (beta / np.abs(beta))   

    orientation = np.array([0.0,0.0,0.0])

    orientation[0] = alpha
    orientation[1] = beta
    orientation[2] = gamma

    return orientation

def GetTransformationFromPoint(point):

    """get transformation matrix from position and orientation of point
    
    :param point: the orientation and position of a point
    :type array of floats
    :returns: the transformation matrix of the point
    :rtype: matrix of floats
    """    

    transformation = np.empty([4, 4])

    transformation[0,3] = point[0]
    transformation[1,3] = point[1]
    transformation[2,3] = point[2]
    transformation[3,3] = 1
    
    transformation[3,0] = 0
    transformation[3,1] = 0
    transformation[3,2] = 0
    
    alpha = point[3]
    beta = point[4]
    gamma = point[5]
    
    transformation[0,0] = np.cos(alpha)*np.cos(beta)
    transformation[0,1] = np.cos(alpha)*np.sin(beta)*np.sin(gamma)-np.sin(alpha)*np.cos(gamma)
    transformation[0,2] = np.cos(alpha)*np.sin(beta)*np.cos(gamma)+np.sin(alpha)*np.sin(gamma)
    transformation[1,0] = np.sin(alpha)*np.cos(beta)
    transformation[1,1] = np.sin(alpha)*np.sin(beta)*np.sin(gamma)+np.cos(alpha)*np.cos(gamma)
    transformation[1,2] = np.sin(alpha)*np.sin(beta)*np.cos(gamma)-np.cos(alpha)*np.sin(gamma)
    transformation[2,0] = -1*np.sin(beta)
    transformation[2,1] = np.cos(beta)*np.sin(gamma)
    transformation[2,2] = np.cos(beta)*np.cos(gamma)

    return transformation

def CalculateInverseKinematics(tcp, one_solution = 8):

    """calculate inverse kinematics for all angles or one solution
    
    :param tcp: the tool center point
    :type array of floats
    :param one_solution: all solutions or one? default: 8 = all solutions
    :type int
    :returns: all possible solutions of inverse kinematics or one
    :rtype: matrix of floats
    """    

    #global joint_one_low
    #global joint_one_high
    #global joint_two_low
    #global joint_two_high
    #global joint_three_low
    #global joint_three_high
    
    position_exists = 0;

    target = np.empty([8,6])
    
    #fill target matrix with pi*2, every solution with pi*2 in it doesn't fullfil angle constraints  
    target.fill(np.pi*2)
    
    angle_buffer = np.array([0.0,0.0])
    
    orientation_buffer = np.empty([2,3])

    #set wrist offset
    wrist_offset = np.array([[0],[0],[-0.158],[1]])
    
    wrist= np.array([[0.0],[0.0],[0.0],[0.0]])
    
    #calculate destination point for wrist postion
    wrist = np.mat(tcp)*np.mat(wrist_offset)

    #transformation of destination point from world to first cs
    working_point_one = CalculateInverseKinematicsTransformation(wrist, 0.0, 1, 0)

    #get first angles
    angle_buffer = CalculateInverseKinematicsAngleOne(working_point_one)

    target[0,0] = angle_buffer[0]
    target[1,0] = angle_buffer[0]
    target[2,0] = angle_buffer[0]
    target[3,0] = angle_buffer[0]
    target[4,0] = angle_buffer[1]
    target[5,0] = angle_buffer[1]
    target[6,0] = angle_buffer[1]
    target[7,0] = angle_buffer[1]

    #calculate all possible solutions
    if(one_solution == 8):

        for i in range(0,2):
        
            #if(joint_one_low<=target[i*4,0]<=joint_one_high):
                
            #transformation of destination point from first cs to second cs
            working_point_two = CalculateInverseKinematicsTransformation(working_point_one, target[i*4,0], 2, 1)  
            
            #get second angles
            angle_buffer = CalculateInverseKinematicsAngleTwo(working_point_two)
    
            target[i*4,1] = angle_buffer[0]
            target[i*4+1,1] = angle_buffer[0]
            target[i*4+2,1] = angle_buffer[1]
            target[i*4+3,1] = angle_buffer[1]
            
            for j in range(0,2):
            
                #if(joint_two_low<=target[i*4+j*2,1]<=joint_two_high):
                
                #transformation of destination point from second cs to third cs
                working_point_three = CalculateInverseKinematicsTransformation(working_point_two, target[i*4+j*2,1], 3, 2)   
            
                #get third angles
                target[i*4+j*2,2] = CalculateInverseKinematicsAngleThree(working_point_three)
                target[i*4+j*2+1,2] = target[i*4+j*2,2]
                
                #if(joint_three_low<=target[i*4+j*2,2]<=joint_three_high):
                
                #get last three angles
                orientation_buffer = CalculateInverseKinematicsOrientation(tcp, np.array([target[i*4+j*2,0],target[i*4+j*2,1],target[i*4+j*2,2]]))
                
                for k in range(0,2):
                
                    target[i*4+j*2+k,3] = orientation_buffer[k,0]
                    target[i*4+j*2+k,4] = orientation_buffer[k,1]
                    target[i*4+j*2+k,5] = orientation_buffer[k,2]
                                
    else:

        #same calculation steps for just one solution

        #if(joint_one_low<=target[one_solution,0]<=joint_one_high):
            
        working_point_two = CalculateInverseKinematicsTransformation(working_point_one, target[one_solution,0], 2, 1)  
        
        angle_buffer = CalculateInverseKinematicsAngleTwo(working_point_two)

        if one_solution%2 == 0:
            target[one_solution,1] = angle_buffer[0]
        else:
            target[one_solution,1] = angle_buffer[1]

        #if(joint_two_low<=target[one_solution,1]<=joint_two_high):
        
        working_point_three = CalculateInverseKinematicsTransformation(working_point_two, target[one_solution,1], 3, 2)   
    
        target[one_solution,2] = CalculateInverseKinematicsAngleThree(working_point_three)
        
        #if(joint_three_low<=target[one_solution,2]<=joint_three_high):
        
        orientation_buffer = CalculateInverseKinematicsOrientation(tcp, np.array([target[one_solution,0],target[one_solution,1],target[one_solution,2]]))
        
        if one_solution%2 == 0:
            target[one_solution,3] = orientation_buffer[0,0]
            target[one_solution,4] = orientation_buffer[0,1]
            target[one_solution,5] = orientation_buffer[0,2]  
        else:      
            target[one_solution,3] = orientation_buffer[1,0]
            target[one_solution,4] = orientation_buffer[1,1]
            target[one_solution,5] = orientation_buffer[1,2]  

    return target

def CalculateInverseKinematicsAngleOne(point):

    """calculate first angle (rotation of robot)
    
    :param point: the wrist point
    :type array of floats
    :returns: the possible angles
    :rtype: array of floats
    """    
    angles = np.array([0.0,0.0])
    
    angle = 0
    
    if not(point[1] == 0 and point[0] == 0):
        angle = np.arctan2(point[1],point[0])
    
    angles[0] = angle
    
    if angle < 0:
        angle += np.pi
    else:
        angle -= np.pi
        
    angles[1] = angle

    return angles

def CalculateInverseKinematicsAngleTwo(point):

    """calculate second angle (shoulder)
    
    :param point: the wrist point
    :type array of floats
    :returns: the possible angles
    :rtype: array of floats
    """    
    
    angles = np.array([0.0,0.0])
    
    angle = 0
    
    d = 0
    
    arccos_input = 0
    
    if not(point[1] == 0 and point[0] == 0):
        angle = np.arctan2(point[1],point[0])
    
        d = np.sqrt(np.power(point[0],2) + np.power(point[1],2))
        
        arccos_input = (np.power(w,2) - np.power(d,2) - np.power(dhParameter[2,0],2)) / (-2 * d * np.abs(dhParameter[2,0]))
    
    #check if point is reachable or too far away
    if not(-1 <= arccos_input <= 1):
        angles[0] = np.pi*2
        angles[1] = np.pi*2
        return angles
    
    angle_mod = np.arccos(arccos_input)
    
    angles[0] = angle - angle_mod
    angles[1] = angle + angle_mod
    
    for i in range(0,2):
        if angles[i] < -np.pi:
            angles[i] += np.pi*2
        elif angles[i] > np.pi:
            angles[i] -= np.pi*2

    return angles

def CalculateInverseKinematicsAngleThree(point):

    """calculate third angle (elbow)
    
    :param point: the wrist point
    :type array of floats
    :returns: the possible angles
    :rtype: array of floats
    """      
    
    angle = 0
    
    if not(point[1] == 0 and point[0] == 0):
        angle = theta_offset + np.arctan2(point[1],point[0])

    return angle

def CalculateInverseKinematicsOrientation(tcp, angles):

    """calculation of the inverse kinematics orientation problem and combination with positioning problem
    
    :param tcp: the tool center point
    :type array of floats
    :param angles: the first three angles
    :type array of floats
    :returns: the possible angles
    :rtype: array of floats
    """      

    #global joint_four_low
    #global joint_four_high
    #global joint_five_low
    #global joint_five_high
    #global joint_six_low
    #global joint_six_high

    target = np.empty([2,3])

    target.fill(np.pi*2)

    orient_mat = np.empty([4, 4])
    
    center_of_cs = np.array([[1.0,0.0,0.0,0.0],
                             [0.0,1.0,0.0,0.0],
                             [0.0,0.0,1.0,0.0],
                             [0.0,0.0,0.0,1.0]])
    
    angle_buffer = np.array([0.0,0.0])
    
    #calculate matrix for orientation angles
    orient_mat = np.mat(tcp)*np.mat(CalculateInverseKinematicsTransformation(center_of_cs, 0.0, 6, 5))                          #transform tool-center-point to wrist point
    orient_mat = np.mat(CalculateInverseKinematicsTransformation(center_of_cs, 0.0, 1, 0))*orient_mat
    orient_mat = np.mat(CalculateInverseKinematicsTransformation(center_of_cs, angles[0], 2, 1))*orient_mat
    orient_mat = np.mat(CalculateInverseKinematicsTransformation(center_of_cs, angles[1], 3, 2))*orient_mat
    orient_mat = np.mat(CalculateInverseKinematicsTransformation(center_of_cs, angles[2], 4, 3))*orient_mat
    
    #get fourth angles
    angle_buffer = CalculateInverseKinematicsAngleFour(orient_mat)
    
    #if(joint_four_low<=angle_buffer[0]<=joint_four_high or joint_four_low<=angle_buffer[1]<=joint_four_high):
    
    target[0,0] = angle_buffer[0]
    target[1,0] = angle_buffer[1]

    #get fifth angles
    angle_buffer = CalculateInverseKinematicsAngleFive(orient_mat)
    
    #if(joint_five_low<=angle_buffer[0]<=joint_five_high or joint_five_low<=angle_buffer[1]<=joint_five_high):

    target[0,1] = angle_buffer[0]
    target[1,1] = angle_buffer[1]

    #get sixth angles
    angle_buffer = CalculateInverseKinematicsAngleSix(orient_mat)
    
    #if(joint_six_low<=angle_buffer[0]<=joint_six_high or joint_six_low<=angle_buffer[1]<=joint_six_high):

    target[0,2] = angle_buffer[0]
    target[1,2] = angle_buffer[1]

    return target

def CalculateInverseKinematicsAngleFour(orient_mat):

    """calculate fourth angle
    
    :param orient_mat: orientation matrix calculated beforehand
    :type marix of floats
    :returns: the possible angles
    :rtype: array of floats
    """       

    angle_buffer = np.array([0.0,0.0])

    r1 = orient_mat[1,2]
    
    r2 = orient_mat[0,2]

    angle = 0

    if not(r1 == 0 and r2 == 0):
        angle = np.arctan2(r1,r2)

    angle_buffer[0] = angle
    
    angle = 0
    
    if not(r1 == 0 and r2 == 0):
        angle = np.arctan2(-r1,-r2)

    angle_buffer[1] = angle

    return angle_buffer

def CalculateInverseKinematicsAngleFive(orient_mat):

    """calculate fifth angle
    
    :param orient_mat: orientation matrix calculated beforehand
    :type marix of floats
    :returns: the possible angles
    :rtype: array of floats
    """      

    angle_buffer = np.array([0.0,0.0])

    r1 = orient_mat[0,2]
    
    r2 = orient_mat[1,2]
    
    r3 = orient_mat[2,2]
    
    d = np.sqrt(np.power(r1,2)+np.power(r2,2))
    
    angle = 0
    
    if not(d == 0 and r3 == 0):  
        angle = np.arctan2(-d,r3)
    
    angle_buffer[0] = angle
    
    angle = 0
    
    if not(d == 0 and r3 == 0): 
        angle = np.arctan2(d,r3)

    angle_buffer[1] = angle

    return angle_buffer

def CalculateInverseKinematicsAngleSix(orient_mat):

    """calculate sixth angle
    
    :param orient_mat: orientation matrix calculated beforehand
    :type marix of floats
    :returns: the possible angles
    :rtype: array of floats
    """      

    angle_buffer = np.array([0.0,0.0])

    r1 = orient_mat[2,1]
    
    r2 = orient_mat[2,0]

    angle = 0
    
    if not(r1 == 0 and r2 == 0): 
        angle = np.arctan2(r1,-r2)

    angle_buffer[0] = angle
    
    angle = 0
    
    if not(r1 == 0 and r2 == 0): 
        angle = np.arctan2(-r1,r2)

    angle_buffer[1] = angle

    return angle_buffer

def CheckAngleConstraints(angles):

    """check if solution is reachable
    
    :param angles: one mathematical possible angle solution
    :type array of floats
    :returns: is it possible?
    :rtype: boolean
    """    

    global joint_one_low
    global joint_one_high
    global joint_two_low
    global joint_two_high
    global joint_three_low
    global joint_three_high
    global joint_four_low
    global joint_four_high
    global joint_five_low
    global joint_five_high
    global joint_six_low
    global joint_six_high

    if not(joint_one_low<=angles[0]<=joint_one_high):
            return False
    elif not(joint_two_low<=angles[1]<=joint_two_high):
            return False
    elif not(joint_three_low<=angles[2]<=joint_three_high):
            return False
    elif not(joint_four_low<=angles[3]<=joint_four_high):
            return False
    elif not(joint_five_low<=angles[4]<=joint_five_high):
            return False
    elif not(joint_six_low<=angles[5]<=joint_six_high):
            return False
        
    return True


def CheckPosConstraints(position):

    """check if solution is reachable
    
    :param position: position and orientation of the tcp
    :type array of floats
    :returns: is it possible?
    :rtype: boolean
    """    


    #tcp over/on groundplane
    if not(0 <= position[2]):
            return False
    """   already checked in inverse kinematics 
    #tcp is not in the robot base
    elif not(0.5 <= position[0] and 0.5 <= position[1] and 0.5 <= position[2]):
            return False"""
        
    return True