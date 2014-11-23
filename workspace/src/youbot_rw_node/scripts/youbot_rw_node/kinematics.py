#!/usr/bin/env python

import math
import numpy as np
from numpy import sin, cos



class Kinematics:
    def __init__(self):
        pass


    ALMOST_PLUS_ONE=0.9999999
    ALMOST_MINUS_ONE=-0.9999999
    ALMOST_ZERO=0.0000001

    min_angles_ = [math.radians(-169.0),math.radians(-90.0),math.radians(-146.0),math.radians(-102.0),math.radians(-167.0)]
    max_angles_ = [math.radians(169.0),math.radians(65.0),math.radians(151.0),math.radians(102.0),math.radians(167.0)]


    dh=list({'theta':0           ,'d':-0.05   ,'a':-0.23    ,'alpha':0               }, #from write plane to joint_1 (KS0)
            {'theta':0           ,'d':0.147   ,'a':0.033    ,'alpha':math.pi/2       } ,#from KS0 to joint 2 (KS1)
            {'theta':math.pi/2   ,'d':0       ,'a':0.155    ,'alpha':0               }, #from KS1 to joint 3 (KS2)
            {'theta':0           ,'d':0       ,'a':0.135    ,'alpha':0               }, #from KS2 to joint 4 (KS3)
            {'theta':math.pi/2   ,'d':0       ,'a':0        ,'alpha':math.pi/2       }, #from KS3 to joint 5 (KS4)
            {'theta':0           ,'d':0.2175  ,'a':0        ,'alpha':0               }, #from KS4 to tcp
            {'theta':0           ,'d':0.03    ,'a':0        ,'alpha':0               }) #from tcp to pencil


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

    def offset2world(self, thetas):
        """ todo

        :param todo
        :type todo
        :returns: todo
        :rtype: todo
        """
        return -thetas

    def direct_kin(self, thetas):
        """ todo

        :param todo
        :type todo
        :returns: todo
        :rtype: todo
        """
        trans= self.get_dh_transform(self.dh[1],thetas[0]) * self.get_dh_transform(self.dh[2],thetas[1]) * \
               self.get_dh_transform(self.dh[3],thetas[2]) * self.get_dh_transform(self.dh[4],thetas[3]) * \
               self.get_dh_transform(self.dh[5],thetas[4])
        return (trans*np.matrix((0,0,0,1)).transpose())[:3]
