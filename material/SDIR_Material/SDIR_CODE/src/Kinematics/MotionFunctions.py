import numpy as np
import time
import math
import KinematicFunctions as kf

glob_traj_path_handles = []

glob_asynch_color = np.array(((0.8,0.4,0.4),(0.8,0.4,0.4)))
glob_asynch_color_high = np.array(((1,0.6,0.6),(1,0.6,0.6)))
glob_synch_color = np.array(((0.4,0.8,0.4),(0.4,0.8,0.4)))
glob_synch_color_high = np.array(((0.6,1,0.6),(0.6,1,0.6)))
glob_lin_color = np.array(((0.4,0.4,0.8),(0.4,0.4,0.8)))
glob_lin_color_high = np.array(((0.6,0.6,1),(0.6,0.6,1)))

glob_singularitiesMat=[]

#maximal velocity for joint 1 - 6 in rad/s
max_velocity = np.array([1.74532925, 1.3962634, 1.3962634, 4.01425728, 2.87979327, 4.34586984])

#maximal acceleration for joint 1 - 6 in rad/s*s
max_acceleration = np.array([4.36332313, 4.36332313, 4.36332313, 7.15584993, 7.15584993, 11.5191731])

#maximal intended velocity for triangle profile
max_intended_velocity = 15.0

#calculations per second TODO correct value?
calc_frequency = 100.0

#time step for one robot calculation
time_step = 1/calc_frequency

def Move(env, robot, trajectory, traj_points, motiontype, drawPath, autoClearPath, showPathStepsPoint, showPathStepsHigh):
    global glob_traj_path_handles
    global glob_synch_color
    global glob_synch_color_high
    global glob_asynch_color
    global glob_asynch_color_high
    global glob_lin_color
    global glob_lin_color_high
    global glob_singularitiesMat
    
    if autoClearPath is True:
        glob_traj_path_handles = []
            
    #glob_singularitiesMat[30][0] = 1
    
    #glob_singularitiesMat[60][1] = 1
    #glob_singularitiesMat[60][0] = 1    
    
    # set path colors
    if motiontype == "S":
        path_colors = glob_synch_color
        path_colors_high = glob_synch_color_high
    elif motiontype == "A":
        path_colors = glob_asynch_color
        path_colors_high = glob_asynch_color_high
    elif motiontype == "L":
        path_colors = glob_lin_color
        path_colors_high = glob_lin_color_high
    
 
    temp_start_angles = np.array([robot.GetDOFValues()])
    start_point = calcTrajPoints(temp_start_angles)
    
    for i in range(trajectory.shape[0]):
        
        highlighted = i % 2
        
        #check if angles are in the constraints; break if not
        temp_ourAngles = kf.SubstractOffsetToAngles(trajectory[i])
        reachableAngles = kf.CheckAngleConstraints(temp_ourAngles)
        reachablePos = kf.CheckPosConstraints(traj_points[i])
        if reachablePos is True:
            if reachableAngles is True:       
                robot.SetDOFValues(trajectory[i])
                if drawPath is True:
                    if i == 0:
                        line_array = np.array(((start_point[0,0],start_point[0,1],start_point[0,2]),(traj_points[i,0],traj_points[i,1],traj_points[i,2])))
                        step_point = np.array(((traj_points[i,0],traj_points[i,1],traj_points[i,2])))
                    else:
                        line_array = np.array(((traj_points[i-1,0],traj_points[i-1,1],traj_points[i-1,2]),(traj_points[i,0],traj_points[i,1],traj_points[i,2]))) 
                        step_point = np.array(((traj_points[i,0],traj_points[i,1],traj_points[i,2])))        
                                    
                    if showPathStepsHigh is True:
                        if highlighted == 0:
                            glob_traj_path_handles.append(env.drawlinestrip(points=line_array,linewidth=2.0,colors=path_colors))
                        elif highlighted == 1:
                            glob_traj_path_handles.append(env.drawlinestrip(points=line_array,linewidth=2.0,colors=path_colors_high))
                    else: 
                        glob_traj_path_handles.append(env.drawlinestrip(points=line_array,linewidth=2.0,colors=path_colors))
                    
                    if showPathStepsPoint is True:
                        glob_traj_path_handles.append(env.plot3(points=step_point,pointsize=2.0,colors=np.array(((0,0,0,1)))))
                                        
                    # check for  over head singularities in joint 0
                    if glob_singularitiesMat[i][0] == 1:
                        point_array = np.array((traj_points[i,0],traj_points[i,1],traj_points[i,2]))
                        glob_traj_path_handles.append(env.plot3(points=point_array,pointsize=8.0,colors=np.array(((1,0,0,0.7)))))
                    # check for wrist singularities in joint 3
                    if glob_singularitiesMat[i][1] == 1:
                        point_array = np.array((traj_points[i,0],traj_points[i,1],traj_points[i,2]))
                        glob_traj_path_handles.append(env.plot3(points=point_array,pointsize=6.0,colors=np.array(((0,1,0,0.7)))))
                    # check for wrist singularities in joint 5
                    if glob_singularitiesMat[i][2] == 1:
                        point_array = np.array((traj_points[i,0],traj_points[i,1],traj_points[i,2]))
                        glob_traj_path_handles.append(env.plot3(points=point_array,pointsize=4.0,colors=np.array(((0,0,1,0.7)))))
                    
                time.sleep(time_step)
            else:
                print "|WARNING|: Angle constraints were hurt! Target is not reachable! Movement was interrupted."
                return
        else:
            print "|WARNING|: Position constraints were hurt! Target is not reachable! Movement was interrupted."
            return

def calcTrajPoints(trajectory_angles):
    """ calculate the trajectory points for given angles of the trajectory 
    
    :param trajectory_angles: array of angles of the trajectory
    :type trajectory_angles: array of floats
    :returns: Array containing the points of the interpolated path
    :rtype: matrix of floats
    """
    
    traj_points = np.empty([trajectory_angles.shape[0], 3])
    tcp_origin = np.array([[0], [0], [0], [1]])
    
    
    for i in range(trajectory_angles.shape[0]):
        temp_our_angle = kf.SubstractOffsetToAngles(trajectory_angles[i])
        tmp_points = kf.CalculateDirectKinematicsTransformation(temp_our_angle, tcp_origin, 0, 7)
        traj_points[i,0] = tmp_points[0] 
        traj_points[i,1] = tmp_points[1]
        traj_points[i,2] = tmp_points[2]
    
    return traj_points
    

def PTPtoConfiguration(start_cfg, target_cfg, motiontype):
    """PTP path planning
    
    :param start_cfg: Current axis angle of the robot
    :type start_cfg: array of floats
    :param target_cfg: Target angle of the robot
    :type target_cfg: array of floats
    :param motiontype: Type of motion (asynchronous, synchronous, fully synchronous)
    :type motiontype: int
    :returns: Array containing the axis angles of the interpolated path
    :rtype: matrix of floats
    """
    
    # Init  
    global glob_singularitiesMat
    delta_angles = target_cfg-start_cfg
    sign = np.empty([6])
    velocity = np.empty([6])
    t_acceleration = np.empty([6])
    d_acceleration = np.empty([6])
    d_constant = np.empty([6])
    t_constant = np.empty([6])
    t_total = np.empty([6])
    
    # do
    for i in range(0,6):
        if delta_angles[i] >= 0:
            sign[i] = 1
        else:
            sign[i] = -1

        velocity[i] = np.sqrt(max_acceleration[i]*np.abs(delta_angles[i]))
        
        if velocity[i] > max_velocity[i]:
            velocity[i] = max_velocity[i]
        if velocity[i] > max_intended_velocity:
            velocity[i] = max_intended_velocity
            
        t_acceleration[i] = velocity[i]/max_acceleration[i]    
        d_acceleration[i] = (max_acceleration[i]*np.power(t_acceleration[i],2))/2
        
        d_constant[i] = np.abs(np.abs(delta_angles[i])-2*d_acceleration[i])    
        if(velocity[i] > 0):
            t_constant[i] = d_constant[i]/velocity[i]
        else:
            t_constant[i] = 0
            
        t_total[i] = 2*t_acceleration[i] + t_constant[i]
        
    total_time = np.max(t_total)
    
    if motiontype == 'S': 
        for i in range(0,6):  
            velocity[i] = (total_time * max_acceleration[i] - np.sqrt( np.power(total_time,2) * np.power(max_acceleration[i],2) - 4 * max_acceleration[i] * np.abs(delta_angles[i]))) / 2
        
            if velocity[i] > max_velocity[i]:
                velocity[i] = max_velocity[i]
            if velocity[i] > max_intended_velocity:
                velocity[i] = max_intended_velocity
                
            t_acceleration[i] = velocity[i]/max_acceleration[i]            
            d_acceleration[i] = (max_acceleration[i]*np.power(t_acceleration[i],2))/2
        
            d_constant[i] = np.abs(np.abs(delta_angles[i])-2*d_acceleration[i])            
            if(velocity[i] > 0):
                t_constant[i] = d_constant[i]/velocity[i]
            else:
                t_constant[i] = 0
        
    step_size = np.int(math.ceil(total_time * calc_frequency))
    
    singularitiesMat = np.empty([step_size, 3]) # matrix of singularities on the trajectory; index 0 for wrist singularities and index 1 for overhead singularities
    singularitiesMat.fill(0)
    
    # init
    trajectory = np.empty([step_size, 6])
    current_T = 0
    current_V = np.array([[0.0,0.0],[0.0,0.0],[0.0,0.0],[0.0,0.0],[0.0,0.0],[0.0,0.0]])
    delta_angle = 0
    
    for i in range(0,step_size-1):
        current_T += time_step        
        for j in range(0,6):
            delta_time_acc = current_T - t_acceleration[j]
            delta_time_dec = current_T - (t_acceleration[j] + t_constant[j])

            #interpolation from acc to acc
            if delta_time_acc <= 0:
                current_V[j,1] = current_V[j,0] + max_acceleration[j] * time_step
            #interpolation from acc to const
            elif delta_time_acc < time_step:
                current_V[j,1] = current_V[j,0] + max_acceleration[j] * (time_step-delta_time_acc)
            #interpolation from const to dec
            elif ((delta_time_dec > 0) and (delta_time_dec < time_step) and (current_T < total_time)):
                current_V[j,1] = np.maximum(current_V[j,0]-max_acceleration[j] * delta_time_dec, 0)
            #interpolation from dec to dec
            elif ((delta_time_dec >= time_step) and (current_T < total_time)):
                current_V[j,1] = np.maximum(current_V[j,0]-max_acceleration[j] * time_step, 0)
        
            #interpolation from const to const when current_V was not modified from previous conditions
            if i > 0:
                delta_angle = sign[j] * current_V[j,1]*time_step
                trajectory[i,j] = trajectory[i-1,j] + delta_angle          
            else:
                delta_angle = sign[j] * current_V[j,1]*time_step
                trajectory[i,j] = start_cfg[j] + delta_angle
                
            current_V[j,0] = current_V[j,1]

    trajectory[step_size-1] = target_cfg
    
    glob_singularitiesMat = singularitiesMat
    
    return trajectory

def LINtoConfiguration(start_cfg, target_cfg, max_vel, max_acc, robot, ignoreSing):
    """LIN path planning
    
    :param start_cfg: Current axis angle of the robot
    :type start_cfg: array of floats
    :param target_cfg: Target angle of the robot
    :type target_cfg: array of floats
    :param max_vel: maximal velocity of end effector in meter per second
    :type max_vel: float
    :param max_acc: maximal acceleration of end effector in meter per second*second
    :type max_acc: float
    :param robot: the robot
    :type robot: kin body
    :returns: LIN trajectory
    :rtype: matrix of floats
    """
    
    print "LIN calculation in progress..."
    
    # INIT
    global glob_singularitiesMat
    singularity = 0 
    selected_solution = 0
    normal_vector = np.empty([3])
    center_of_cs = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
    
    # get rid of offset
    start_cfg_our = kf.SubstractOffsetToAngles(start_cfg)   
    target_cfg_our = kf.SubstractOffsetToAngles(target_cfg)
    
    # calc start and target point
    start_point = kf.CalculateDirectKinematicsTransformation(start_cfg_our, center_of_cs, 0, 7)  
    target_point = kf.CalculateDirectKinematicsTransformation(target_cfg_our, center_of_cs, 0, 7)
    
    # length of path  
    length = np.sqrt(np.power(target_point[0,3]-start_point[0,3],2)+np.power(target_point[1,3]-start_point[1,3],2)+np.power(target_point[2,3]-start_point[2,3],2))

    if np.around(length, 5) == 0:
        print "Target point is equal to start point. Linear movement is not possible. Choose another target point!"
        return np.array([robot.GetDOFValues()])
    
    # length of the maximal acceleration time both phases
    length_A_Max = np.power(max_vel,2)/max_acc
    
    # check if acceleration length is bigger then path length
    if length_A_Max >= length:
        max_vel = np.sqrt(max_acc * length)
        
    # calc time and distance for one acceleration phase    
    t_acceleration = max_vel/max_acc
    d_acceleration = np.power(max_vel,2)/(2*max_acc)
    
    # calc distance and time for the constant velocity movement
    d_constant = length - 2 * d_acceleration
    t_constant = d_constant/max_vel
    
    total_time = t_acceleration * 2 + t_constant
    step_size = np.int(math.ceil(total_time * calc_frequency))
    
    # init trajectory and singularities matrix
    trajectory = np.empty([step_size, 6])
    singularitiesMat = np.empty([step_size, 3]) # matrix of singularities on the trajectory; index 0 for wrist singularities and index 1 for overhead singularities
    singularitiesMat.fill(0) 
    
    # calc normal vector
    normal_vector[0] = (target_point[0,3]-start_point[0,3])/length
    normal_vector[1] = (target_point[1,3]-start_point[1,3])/length
    normal_vector[2] = (target_point[2,3]-start_point[2,3])/length
    
    current_T = 0
    current_S = 0
    current_P = np.empty([4,4])
    
    singularity_list = np.empty([0])
    singularity_type_list = np.empty([0])
    # go through path  from start till 1 step before end
    for i in range(0,step_size-1):
        current_T += time_step
        
        #interpolation from acc to acc
        if current_T - t_acceleration <= 0:
            current_S = max_acc * np.power(current_T,2)/2
        #interpolation from acc to dec
        elif current_T - t_acceleration < time_step and t_constant <= 0:
            current_S = max_acc * np.power(current_T-(current_T - t_acceleration),2)/2
            current_S += (max_vel*(current_T - (t_acceleration + t_constant))-max_acc*np.power(current_T - (t_acceleration + t_constant),2))/2
        #interpolation from acc to const
        elif current_T - t_acceleration < time_step:
            current_S = max_acc * np.power(current_T-(current_T - t_acceleration),2)/2
            current_S = current_S + max_vel*(current_T - t_acceleration)
        #interpolation from const to const
        elif current_T - (t_acceleration + t_constant) <= 0:
            current_S = current_S + max_vel*time_step
        #interpolation from const to dec
        elif current_T - (t_acceleration + t_constant) < time_step:
            current_S = current_S + max_vel*(time_step -(current_T - (t_acceleration + t_constant)))
            current_S = current_S + (max_vel*(current_T - (t_acceleration + t_constant))-max_acc*np.power(current_T - (t_acceleration + t_constant),2))/2
        #interpolation from dec to dec
        elif current_T < total_time:
            current_S = current_S + (max_vel*time_step-max_acc*np.power(time_step,2))/2
    
        # calc target point for the current step 
        current_P = np.copy(start_point)
        current_P[0,3] = start_point[0,3] + normal_vector[0] * current_S
        current_P[1,3] = start_point[1,3] + normal_vector[1] * current_S
        current_P[2,3] = start_point[2,3] + normal_vector[2] * current_S
        
        # calc solutions for current step
        solutions = kf.CalculateInverseKinematics(current_P)
        
        # do only for the first step
        if(i == 0):            
            #calculate quadratic error to find solution which fits the previous configuration
            selected_solution = calcIkSolutionError(solutions, start_cfg_our, False)    
        
        # do for the other steps 
        else:        
            #calculate quadratic error to find solution which fits the previous configuration
            selected_solution = calcIkSolutionError(solutions, trajectory[i-1], True) 
  
        # if a solution is possible
        if not (selected_solution == -1):
            trajectory[i] = kf.AddOffsetToAngles(solutions[selected_solution])
        else:
            print "No possible path found. inverse kinematics got no reachable solution."
            return np.array([robot.GetDOFValues()])
        
        # check of singularities
        if i >= 1:
        
            delta_angle_joint_one = np.abs(trajectory[i-1][0] - trajectory[i][0])
            delta_angle_joint_four = np.abs(trajectory[i-1][3] - trajectory[i][3])
            delta_angle_joint_six = np.abs(trajectory[i-1][5] - trajectory[i][5])
        
            #singularity if angles exceed maximum velocity in one calculation step
            #for wrist singularity
            if delta_angle_joint_four > max_velocity[3]*time_step or delta_angle_joint_six > max_velocity[5]*time_step:
                
                #position of singularity
                singularity_point = np.array([i])
                singularity_list = np.append(singularity_list, singularity_point)
                singularity_type = np.array([1])
                singularity_type_list = np.append(singularity_type_list, singularity_type)
                
                if delta_angle_joint_four > max_velocity[3]*time_step:
                    singularitiesMat[i][1] = 1
                
                if delta_angle_joint_six > max_velocity[5]*time_step:
                    singularitiesMat[i][2] = 1
            
            #for overhead singularity
            if delta_angle_joint_one > max_velocity[0]*time_step:
            
                #position of singularity
                singularity_point = np.array([i])
                singularity_list = np.append(singularity_list, singularity_point)
                singularity_type = np.array([0])
                singularity_type_list = np.append(singularity_type_list, singularity_type)
                singularitiesMat[i][0] = 1
    
    # last step
    last_trgt = np.copy(start_point)
    last_trgt[0,3] = target_point[0,3]
    last_trgt[1,3] = target_point[1,3]
    last_trgt[2,3] = target_point[2,3]
    
    solutions = kf.CalculateInverseKinematics(last_trgt)

    #calculate quadratic error to find solution which fits the previous configuration
    selected_solution = calcIkSolutionError(solutions, trajectory[step_size-2], True)
    
    # if a solution is possible
    if not (selected_solution == -1):
        trajectory[step_size-1] = kf.AddOffsetToAngles(solutions[selected_solution])
    else:
        print "No possible path found. inverse kinematics got no reachable solution."
        return np.array([robot.GetDOFValues()])
    
    if not ignoreSing:
        finished_trajectory = handleSingularity(singularity_list, singularity_type_list, trajectory, robot)
    else:
        finished_trajectory = np.copy(trajectory)
    
    for i in range(finished_trajectory.shape[0]):
        if not kf.CheckAngleConstraints(finished_trajectory[i]):
            print "No possible path found because of angle constraints violation."
            return np.array([robot.GetDOFValues()])
    
    
    glob_singularitiesMat = singularitiesMat
    
    return finished_trajectory

def calcIkSolutionError(solutions, trajectory_step, subtractOffset):
    #init
    smallest_error = 10000.0
    selected_solution = -1
    quadratic_error = np.empty([8])     
    quadratic_error.fill(0.0)
    
    #do
    for j in range(0,8):             
        for k in range(0,6):
            if subtractOffset is True:                
                reference_trajectory = kf.SubstractOffsetToAngles(trajectory_step)
            else:
                reference_trajectory = trajectory_step
                                
            quadratic_error[j] += np.abs(solutions[j][k] - reference_trajectory[k])
            
        if quadratic_error[j] < smallest_error:                
            smallest_error = quadratic_error[j]
            selected_solution = j
    
    return selected_solution

def handleSingularity(singularity_list, singularity_type, trajectory, robot):
    """ try to handle all occuredsingularities
    
    :param singularity_list: array of indices where singularities occured
    :type singularity_list: array of floats
    :param singularity_type: array of type of singularity (index of singularity_list and singularity_type correspond)
    :type singularity_type: array of floats
    :param trajectory: matrix of angles in every step of the trajectory
    :type trajectory: matrix of floats
    :param robot: the robot
    :type robot: kinbody
    :returns: Matrix of angles in every step of the trajectory
    :rtype: matrix of floats
    """
    
    multiStepInter = False

    target_trajectory = np.copy(trajectory)
    
    if multiStepInter is False:
        for i in range(0,np.size(singularity_list)):
            
            index = singularity_list[i]
            
            if(singularity_type[i] == 0):
                
                target_trajectory[index][0] = (trajectory[index-1][0] + trajectory[index+1][0]) / 2
                target_trajectory[index][1] = (trajectory[index-1][1] + trajectory[index+1][1]) / 2
                target_trajectory[index][2] = (trajectory[index-1][2] + trajectory[index+1][2]) / 2
                target_trajectory[index][3] = (trajectory[index-1][3] + trajectory[index+1][3]) / 2
                target_trajectory[index][4] = (trajectory[index-1][4] + trajectory[index+1][4]) / 2
                target_trajectory[index][5] = (trajectory[index-1][5] + trajectory[index+1][5]) / 2
                       
                for j in range(0,6):
                    for k in range(0,2):    
                         
                        delta_angle_joint = np.abs(target_trajectory[index-1+k][j] - trajectory[index+k][j])
                        
                        if delta_angle_joint > max_velocity[j]*time_step:                         
                                print "No possible path found because of overhead singularity."
                                return np.array([robot.GetDOFValues()])
                
            if(singularity_type[i] == 1):
        
                target_trajectory[index][0] = (trajectory[index-1][0] + trajectory[index+1][0]) / 2
                target_trajectory[index][1] = (trajectory[index-1][1] + trajectory[index+1][1]) / 2
                target_trajectory[index][2] = (trajectory[index-1][2] + trajectory[index+1][2]) / 2
                target_trajectory[index][3] = (trajectory[index-1][3] + trajectory[index+1][3]) / 2
                target_trajectory[index][4] = (trajectory[index-1][4] + trajectory[index+1][4]) / 2
                target_trajectory[index][5] = (trajectory[index-1][5] + trajectory[index+1][5]) / 2
        
                for j in range(0,6):
                    for k in range(0,2):    
                         
                        delta_angle_joint = np.abs(target_trajectory[index-1+k][j] - trajectory[index+k][j])
                        
                        if delta_angle_joint > max_velocity[j]*time_step:                         
                                print "No possible path found because of wrist singularity."
                                print 
                                return np.array([robot.GetDOFValues()])         
    else:
        # Multi step interpolation dont function yet
        interStep = 2
        InterSuccess = False
        
        while InterSuccess is False:
        
            for i in range(0,np.size(singularity_list)):
                
                index = singularity_list[i]
                
                #Todo: check if interpolation width hurts trajectory range or other singularity interpolations             
                    
                target_trajectory[index][0] = (target_trajectory[index-(interStep/2)][0] + target_trajectory[index+(interStep/2)][0]) / 2
                target_trajectory[index][1] = (target_trajectory[index-(interStep/2)][1] + target_trajectory[index+(interStep/2)][1]) / 2
                target_trajectory[index][2] = (target_trajectory[index-(interStep/2)][2] + target_trajectory[index+(interStep/2)][2]) / 2
                target_trajectory[index][3] = (target_trajectory[index-(interStep/2)][3] + target_trajectory[index+(interStep/2)][3]) / 2
                target_trajectory[index][4] = (target_trajectory[index-(interStep/2)][4] + target_trajectory[index+(interStep/2)][4]) / 2
                target_trajectory[index][5] = (target_trajectory[index-(interStep/2)][5] + target_trajectory[index+(interStep/2)][5]) / 2
                
                delta_traj = target_trajectory[index]
                delta_traj[0] = (target_trajectory[index+(interStep/2)][0] - target_trajectory[index-(interStep/2)][0]) / interStep
                delta_traj[1] = (target_trajectory[index+(interStep/2)][1] - target_trajectory[index-(interStep/2)][1]) / interStep
                delta_traj[2] = (target_trajectory[index+(interStep/2)][2] - target_trajectory[index-(interStep/2)][2]) / interStep
                delta_traj[3] = (target_trajectory[index+(interStep/2)][3] - target_trajectory[index-(interStep/2)][3]) / interStep
                delta_traj[4] = (target_trajectory[index+(interStep/2)][4] - target_trajectory[index-(interStep/2)][4]) / interStep
                delta_traj[5] = (target_trajectory[index+(interStep/2)][5] - target_trajectory[index-(interStep/2)][5]) / interStep
                
                for s in range(1,interStep/2):                            
                    target_trajectory[index-s][0] = target_trajectory[index][0] - (s * delta_traj[0]) 
                    target_trajectory[index-s][1] = target_trajectory[index][1] - (s * delta_traj[1]) 
                    target_trajectory[index-s][2] = target_trajectory[index][2] - (s * delta_traj[2])
                    target_trajectory[index-s][3] = target_trajectory[index][3] - (s * delta_traj[3])
                    target_trajectory[index-s][4] = target_trajectory[index][4] - (s * delta_traj[4])
                    target_trajectory[index-s][5] = target_trajectory[index][5] - (s * delta_traj[5])
                    
                    target_trajectory[index+s][0] = target_trajectory[index][0] + (s * delta_traj[0]) 
                    target_trajectory[index+s][1] = target_trajectory[index][1] + (s * delta_traj[1]) 
                    target_trajectory[index+s][2] = target_trajectory[index][2] + (s * delta_traj[2])
                    target_trajectory[index+s][3] = target_trajectory[index][3] + (s * delta_traj[3])
                    target_trajectory[index+s][4] = target_trajectory[index][4] + (s * delta_traj[4])
                    target_trajectory[index+s][5] = target_trajectory[index][5] + (s * delta_traj[5])
                
                maxVelHurt = False
                
                for j in range(0,6):
                    for k in range(0,interStep):    
                         
                        delta_angle_joint = np.abs(target_trajectory[index-(interStep/2)+k][j] - target_trajectory[index-((interStep/2)-1)+k][j])
                        
                        if delta_angle_joint > max_velocity[j]*time_step and maxVelHurt is False:
                            
                            maxVelHurt = True                     
                            print "Interpolation step = " 
                            print interStep 
                            print "No possible path found because of singularity."
                            print "joint:"
                            print j
                            print "step:"
                            print k
                            
                            print delta_angle_joint
                            print ">"
                            print (max_velocity[j]*time_step)
                            print "______________________________________________"
                            #return np.array([robot.GetDOFValues()])
                            interStep = interStep + 2
                            if interStep == 30:
                                return np.array([robot.GetDOFValues()])
                            
                            
                                                                                    
                if maxVelHurt is False:
                    InterSuccess = True            
                      
                  
        
    return target_trajectory