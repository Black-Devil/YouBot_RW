#!/usr/bin/env python

import math
import numpy as np
from numpy import sin, cos

from numpy import sin, cos, arctan2, arccos
from numpy.core.umath import arccos


class Kinematics:
    def __init__(self):
        pass


    ALMOST_PLUS_ONE=0.9999999
    ALMOST_MINUS_ONE=-0.9999999
    ALMOST_ZERO=0.0000001

    min_angles_ = [math.radians(-169.0),math.radians(-90.0),math.radians(-146.0),math.radians(-102.0),math.radians(-167.0)]
    max_angles_ = [math.radians(169.0),math.radians(65.0),math.radians(151.0),math.radians(102.0),math.radians(167.0)]

    dh=list(({'theta':0.0           ,'d':-0.055   ,'a':0.35    ,'alpha':0.0               }, #from write plane to joint_1 (KS0)
            {'theta':0.0           ,'d':0.147   ,'a':0.033    ,'alpha':math.pi/2       } ,#from KS0 to joint 2 (KS1)
            {'theta':math.pi/2   ,'d':0.0       ,'a':0.155    ,'alpha':0.0               }, #from KS1 to joint 3 (KS2)
            {'theta':0.0           ,'d':0.0       ,'a':0.135    ,'alpha':0.0               }, #from KS2 to joint 4 (KS3)
            {'theta':math.pi/2   ,'d':0.0       ,'a':0.0        ,'alpha':math.pi/2       }, #from KS3 to joint 5 (KS4)
            {'theta':0.0           ,'d':0.2175  ,'a':0.0        ,'alpha':0.0               }, #from KS4 to tcp
            {'theta':0.0           ,'d':0.02    ,'a':0.0        ,'alpha':0.0               })) #from tcp to pencil


    def get_dh_transform(self, dh, theta=0):
        """ todo

        :param todo
        :type todo
        :returns: todo
        :rtype: todo
        """
        trans = np.matrix((    (cos(dh['theta']+theta),    -sin(dh['theta']+theta)*cos(dh['alpha']),       sin(dh['theta']+theta)*sin(dh['alpha']),       dh['a'] *cos(dh['theta']+theta)),
            (sin(dh['theta']+theta),                      cos(dh['theta']+theta)*cos(dh['alpha']),       -cos(dh['theta']+theta)*sin(dh['alpha']),       dh['a']*sin(dh['theta']+theta)),
            (0,                                     sin(dh['alpha']),                         cos(dh['alpha']),                        dh['d']),
            (0,                                     0,                                        0,                                       1)))
        return trans


    def get_inv_transform(self, dh, theta=0):
        """ todo

        :param todo
        :type todo
        :returns: todo
        :rtype: todo
        """
        trans = np.matrix((  (cos(dh['theta']+theta),       sin(dh['theta']+theta),                         0,                                              -dh['a']),
            (-sin(dh['theta']+theta)*cos(dh['alpha']),      cos(dh['theta']+theta)*cos(dh['alpha']),        sin(dh['alpha']),                               -dh['d']*sin(dh['alpha'])),
            (sin(dh['alpha'])*sin(dh['theta']+theta),       -cos(dh['theta']+theta)*sin(dh['alpha']),       cos(dh['alpha']),                               -dh['d']*cos(dh['alpha'])),
            (0,                                             0,                                              0,                                              1)))
        return trans


    def get_transformation2wrist_point(self, alpha):
        dz =(self.dh[5]['d']+self.dh[6]['d'])
        return np.matrix((  (cos(alpha),    0,      sin(alpha),     sin(alpha)*dz),
                            (0  ,           1   ,   0,              0),
                            (-sin(alpha),   0,      cos(alpha),     cos(alpha)*dz),
                            (0,             0,      0,              1)
        ))


    def offset2world(self, thetas):
        """ todo

        :param todo
        :type todo
        :returns: todo
        :rtype: todo
        """
        return -thetas


    def direct_kin(self, thetas, bogen):
        """ todo

        :param todo
        :type todo
        :returns: todo
        :rtype: todo
        """
        if bogen:
            trans=  self.get_dh_transform(self.dh[0],0.0) * \
                    self.get_dh_transform(self.dh[1],thetas[0]) * self.get_dh_transform(self.dh[2],thetas[1]) * \
                    self.get_dh_transform(self.dh[3],thetas[2]) * self.get_dh_transform(self.dh[4],thetas[3]) * \
                    self.get_dh_transform(self.dh[5],thetas[4]) * self.get_dh_transform(self.dh[6],0.0)
        else:
            trans=  self.get_dh_transform(self.dh[0],0.0) * \
                    self.get_dh_transform(self.dh[1],math.radians(thetas[0])) * self.get_dh_transform(self.dh[2],math.radians(thetas[1])) * \
                    self.get_dh_transform(self.dh[3],math.radians(thetas[2])) * self.get_dh_transform(self.dh[4],math.radians(thetas[3])) * \
                    self.get_dh_transform(self.dh[5],math.radians(thetas[4])) * self.get_dh_transform(self.dh[6],0.0)
        return (trans*( np.matrix((0,0,0,1)).transpose() ))[:3]


    def inverse_kin( self, point , condition_angle):
        """ calculate the inverse kinematics

        :param point on write plane, (x;y;z)
        :type todo
        :returns: possible solutions of inverse kinematic
        :rtype: list of( arrays of floats)
        """

        wp= np.matrix(np.resize(point,4)).transpose()
        wp[3]=1                                                         # make homogenous coordinates
        tran = self.get_transformation2wrist_point(math.radians(condition_angle))
        #print "trafo2WP", tran
        wp = self.get_transformation2wrist_point(math.radians(condition_angle)) * wp   # offset calculate Wrist point under condition that Wrist is 45degree up on write plane
        #print "wpoint: [%.4f; %.4f; %.4f; %.4f]" % (wp[0],wp[1],wp[2], wp[3])
        wp_0=self.get_inv_transform(self.dh[0])*wp                    # transform wp into KS0
        #print "wp_0: [%.4f; %.4f; %.4f; %.4f]" % (wp_0[0],wp_0[1],wp_0[2], wp_0[3])
        theta_0 = np.empty([2])
        theta_0[0]=arctan2(wp_0[1],wp_0[0])                 # turn robot arm into wrist point plane

        if theta_0[0] < 0:
            theta_0[1]=theta_0[0]+np.pi
        else:
            theta_0[1]=theta_0[0]-np.pi
        #print "theta_0: [%.4f; %.4f]" % (math.degrees(theta_0[0]),math.degrees(theta_0[1]))

        wp_1 = np.array([self.get_inv_transform(self.dh[1],theta_0[0])*wp_0, self.get_inv_transform(self.dh[1],theta_0[1])*wp_0])       # numpy array of 2 points
        #print "wp_1_0: [%.4f; %.4f; %.4f; %.4f]" % (wp_1[0][0],wp_1[0][1],wp_1[0][2], wp_1[0][3])
        #print "wp_1_1: [%.4f; %.4f; %.4f; %.4f]" % (wp_1[1][0],wp_1[1][1],wp_1[1][2], wp_1[1][3])

        d_wp_1 = np.array(( np.sqrt((wp_1[0][0]**2 + wp_1[0][1]**2)),np.sqrt((wp_1[1][0]**2 + wp_1[1][1]**2)) ))    # array of 2 distances
        #print "d_wp_1: ", d_wp_1

        s=np.array([arctan2(wp_1[0][1],wp_1[0][0]),arctan2(wp_1[1][1],wp_1[1][0])])                                 # array of 2 angles
        #print "s: [%.4f; %.4f]" % (math.degrees(s[0]), math.degrees(s[1]))
        #print "r arcos 0: ", (self.dh[3]['a']**2-d_wp_1[0]**2-self.dh[2]['a']**2) / (-2*d_wp_1[0]*self.dh[2]['a'])
        #print "r arcos 1: ", (self.dh[3]['a']**2-d_wp_1[1]**2-self.dh[2]['a']**2) / (-2*d_wp_1[1]*self.dh[2]['a'])
        r=np.array([arccos( (self.dh[3]['a']**2-d_wp_1[0]**2-self.dh[2]['a']**2) / (-2*d_wp_1[0]*self.dh[2]['a']) ),
                    arccos( (self.dh[3]['a']**2-d_wp_1[1]**2-self.dh[2]['a']**2) / (-2*d_wp_1[1]*self.dh[2]['a']) )])              # array of 2 angles
        #print "r: [%.4f; %.4f]" % (math.degrees(r[0]), math.degrees(r[1]))

        theta_1_0 = np.empty([2])                                                                                   #array of 2 angles
        theta_1_1 = np.empty([2])                                                                                   #array of 2 angles

        theta_1_0[0] =s[0]+r[0]
        theta_1_0[1] =s[0]-r[0]
        theta_1_1[0] =s[1]+r[1]
        theta_1_1[1] =s[1]-r[1]

        # handle wrong turning direction--> make robot turn the shortest direction
        for i in range(0,len(theta_1_0)):
            if theta_1_0[i] < -np.pi:
                theta_1_0[i] += np.pi*2
            elif theta_1_0[i] > np.pi:
                theta_1_0[i] -= np.pi*2
        for i in range(0,len(theta_1_1)):
            if theta_1_1[i] < -np.pi:
                theta_1_1[i] += np.pi*2
            elif theta_1_1[i] > np.pi:
                theta_1_1[i] -= np.pi*2

        #print "theta_1_0: [%.4f; %.4f]" % (math.degrees(theta_1_0[0]), math.degrees(theta_1_0[1]))
        #print "theta_1_1: [%.4f; %.4f]" % (math.degrees(theta_1_1[0]), math.degrees(theta_1_1[1]))

        wp_2=  np.array([self.get_inv_transform(self.dh[2],theta_1_0[0])*wp_1[0], self.get_inv_transform(self.dh[2],theta_1_0[1])*wp_1[0] ,     # numpy array of 4 points
                         self.get_inv_transform(self.dh[2],theta_1_1[0])*wp_1[1], self.get_inv_transform(self.dh[2],theta_1_1[1])*wp_1[1]])

        #print "wp_2: ", wp_2

        theta_2_0 = np.empty([2])                                                                                   #array of 2 angles
        theta_2_1 = np.empty([2])                                                                                   #array of 2 angles

        theta_2_0[0] = arctan2(wp_2[0][1], wp_2[0][0])
        theta_2_0[1] = arctan2(wp_2[1][1], wp_2[1][0])
        theta_2_1[0] = arctan2(wp_2[2][1], wp_2[2][0])
        theta_2_1[1] = arctan2(wp_2[3][1], wp_2[3][0])

        #print "theta_2_0: [%.4f; %.4f]" % (math.degrees(theta_2_0[0]), math.degrees(theta_2_0[1]))
        #print "theta_2_1: [%.4f; %.4f]" % (math.degrees(theta_2_1[0]), math.degrees(theta_2_1[1]))

        wp_3=  np.array([self.get_inv_transform(self.dh[3],theta_2_0[0])*wp_2[0], self.get_inv_transform(self.dh[3],theta_2_0[1])*wp_2[0] ,     # numpy array of 4 points
                         self.get_inv_transform(self.dh[3],theta_2_1[0])*wp_2[1], self.get_inv_transform(self.dh[3],theta_2_1[1])*wp_2[1]])

        theta_3_0 = np.empty([2])                                                                                   #array of 2 angles
        theta_3_1 = np.empty([2])                                                                                   #array of 2 angles

        theta_3_0[0] = arctan2(wp_3[0][1], wp_3[0][0])
        theta_3_0[1] = arctan2(wp_3[1][1], wp_3[1][0])
        theta_3_1[0] = arctan2(wp_3[2][1], wp_3[2][0])
        theta_3_1[1] = arctan2(wp_3[3][1], wp_3[3][0])



        result = list()
        result.append(np.array([ theta_0[0], theta_1_0[0], theta_2_0[0], theta_3_0[0], 0.0 ]))
        result.append(np.array([ theta_0[0], theta_1_0[1], theta_2_0[1], theta_3_0[1], 0.0 ]))
        result.append(np.array([ theta_0[1], theta_1_1[0], theta_2_1[0], theta_3_1[0], 0.0 ]))
        result.append(np.array([ theta_0[1], theta_1_1[1], theta_2_1[1], theta_3_1[1], 0.0 ]))

        for i in result:
            print "[%.4f; %.4f; %.4f; %.4f; %.4f;]" % (math.degrees(i[0]), math.degrees(i[1]), math.degrees(i[2]), math.degrees(i[3]), math.degrees(i[4]) ) , self.isSolutionValid(i)
            #print self.isSolutionValid(i)
        return result


    def isSolutionValid(self, solution):
        """ todo

        :param todo
        :type todo
        :returns: todo
        :rtype: todo
        """
        if len(solution) != 5:
            return False
        for i in range(0,len(solution)):
            if math.isnan(solution[i]):
                return False
            elif solution[i] < self.min_angles_[i] or solution[i] > self.max_angles_[i]:
                return False

        return True


    def get_valid_inverse_kin_solutions(self, point):
        """ todo

        :param todo
        :type todo
        :returns: todo
        :rtype: todo
        """
        #ik_solutions = self.inverse_kin(point, condition_angle)
        condition_angle = np.array([ 45.0, 50.0, 55.0, 60.0, 65.0 , 70.0, 75.0, 80.0, 85.0, 90.0 ])
        ik_solutions = list()
        for i in range(0,len(condition_angle)):
            print "condition: ", condition_angle[i]
            tmpSol =  self.inverse_kin(point, condition_angle[i])
            for k in tmpSol:
                ik_solutions.append(k)

        valid_solutions = list()
        for i in ik_solutions:
            if self.isSolutionValid(i) == True:
                valid_solutions.append(i)

        return valid_solutions
