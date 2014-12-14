#!/usr/bin/env python

import math
import sys
import numpy as np
from numpy import sin, cos
from time import sleep
from numpy import sin, cos, arctan2, arccos
from scipy.optimize import minimize

from numpy.core.umath import arccos


class Kinematics:
    destination_point = np.array([0, 0, 0])
    last_solution = np.array([-4.5284128798882501e-09, 0.84351333295648456, 1.2692013125752315, 0.014612536532375684, 0])
    last_point = np.array([0, 0, 0])
    init_point = np.array([-4.5284128798882501e-09, 0.84351333295648456, 1.2692013125752315, 0.014612536532375684, 0])


    def __init__(self):
        self.last_solution=self.minimize(self.last_solution)
        #print self.last_solution


    min_angles_ = [math.radians(-169.0), math.radians(-90.0), math.radians(-146.0), math.radians(-102.0),
                   math.radians(-167.0)]
    max_angles_ = [math.radians(169.0), math.radians(65.0), math.radians(151.0), math.radians(102.0),
                   math.radians(167.0)]

    dh = list(({'theta': 0.0, 'd': -0.055, 'a': 0.35, 'alpha': 0.0},
               # from write plane to joint_1 (KS0)
               {'theta': 0.0, 'd': 0.147, 'a': 0.033, 'alpha': math.pi / 2},  # from KS0 to joint 2 (KS1)
               {'theta': math.pi / 2, 'd': 0.0, 'a': 0.155, 'alpha': 0.0},  #from KS1 to joint 3 (KS2)
               {'theta': 0.0, 'd': 0.0, 'a': 0.135, 'alpha': 0.0},  #from KS2 to joint 4 (KS3)
               {'theta': math.pi / 2, 'd': 0.0, 'a': 0.0, 'alpha': math.pi / 2},  #from KS3 to joint 5 (KS4)
               {'theta': 0.0, 'd': 0.2175, 'a': 0.0, 'alpha': 0.0},  #from KS4 to tcp
               {'theta': 0.0, 'd': 0.02, 'a': 0.0, 'alpha': 0.0}))  # from tcp to pencil


    def get_dh_transform(self, dh, theta=0):
        trans = np.matrix(((cos(dh['theta'] + theta), -sin(dh['theta'] + theta) * cos(dh['alpha']),
                            sin(dh['theta'] + theta) * sin(dh['alpha']), dh['a'] * cos(dh['theta'] + theta)),
                           (sin(dh['theta'] + theta), cos(dh['theta'] + theta) * cos(dh['alpha']),
                            -cos(dh['theta'] + theta) * sin(dh['alpha']), dh['a'] * sin(dh['theta'] + theta)),
                           (0, sin(dh['alpha']), cos(dh['alpha']), dh['d']),
                           (0, 0, 0, 1)))
        return trans


    def get_inv_transform(self, dh, theta=0):
        trans = np.matrix((  (cos(dh['theta'] + theta), sin(dh['theta'] + theta), 0, -dh['a']),
                             (-sin(dh['theta'] + theta) * cos(dh['alpha']), cos(dh['theta'] + theta) * cos(dh['alpha']),
                              sin(dh['alpha']), -dh['d'] * sin(dh['alpha'])),
                             (sin(dh['alpha']) * sin(dh['theta'] + theta), -cos(dh['theta'] + theta) * sin(dh['alpha']),
                              cos(dh['alpha']), -dh['d'] * cos(dh['alpha'])),
                             (0, 0, 0, 1)))
        return trans


    def direct_kin(self, thetas):
        trans = self.get_dh_transform(self.dh[0], 0.0) * \
                self.get_dh_transform(self.dh[1], thetas[0]) * self.get_dh_transform(self.dh[2], thetas[1]) * \
                self.get_dh_transform(self.dh[3], thetas[2]) * self.get_dh_transform(self.dh[4], thetas[3]) * \
                self.get_dh_transform(self.dh[5], thetas[4]) * self.get_dh_transform(self.dh[6], 0.0)
        return np.array((trans * np.matrix((0, 0, 0, 1)).transpose()).transpose())[0][0:3]


    def direct_kin_wp(self, thetas):
        trans = self.get_dh_transform(self.dh[0], 0.0) * \
                self.get_dh_transform(self.dh[1], thetas[0]) * self.get_dh_transform(self.dh[2], thetas[1]) * \
                self.get_dh_transform(self.dh[3], thetas[2]) * self.get_dh_transform(self.dh[4], thetas[3])
        return np.array((trans * np.matrix((0, 0, 0, 1)).transpose()).transpose())[0][0:3]


    def err_function(self, thetas):
        new_theta = [thetas[0], thetas[1], thetas[2], thetas[3], 0]
        errors = self.direct_kin(new_theta) - self.destination_point
        return (errors ** 2).sum()

    def ist_rund(self, point):
        if abs(point[0] - self.destination_point[0]) > 0.03:
            return False
        if abs(point[1] - self.destination_point[1]) > 0.03:
            return False
        if abs(point[2] - self.destination_point[2]) > 0.03:
            return False
        return True


    def update_progresss(self, progress):
        sys.stdout.write('\r[{0}{1}] {2:3.2f}%'.format('#'*(int(progress/2)),' '*((100/2)-(int(progress/2))) ,progress))
        sys.stdout.flush()


    def search_all_solutions(self, point, resolution):
        resolution = math.fabs(resolution)
        self.destination_point = point
        solutions = list()
        print "Caluclating solutions..."
        print "Resolution: ", resolution

        wp = np.matrix(np.resize(point, 4)).transpose()
        wp[3] = 1  # make homogenous coordinates

        wp_0 = self.get_inv_transform(self.dh[0]) * wp
        theta_0 = np.empty([2])
        theta_0[0] = arctan2(wp_0[1], wp_0[0])
        if theta_0[0] < 0:
            theta_0[1] = theta_0[0] + np.pi
        else:
            theta_0[1] = theta_0[0] - np.pi
        theta0 = list()
        for solution in theta_0:
            if self.isSolutionValid_single(solution, 0):
                theta0.append(solution)

        print "{a} possible solutions".format(a=(155 * 297 * 204 * len(theta0)) / resolution)

        resolution = math.radians(resolution)
        percent = 0
        for i in theta0:
            i1 = self.min_angles_[1]
            while i1 <= self.max_angles_[1]:
                progress=percent+(((i1 - self.min_angles_[1]) / (self.max_angles_[1] - self.min_angles_[1]) * 100)/len(theta0))
                self.update_progresss(progress)
                i2 = self.min_angles_[2]
                while i2 <= self.max_angles_[2]:
                    i3 = self.min_angles_[3]
                    while i3 <= self.max_angles_[3]:
                        thetas = [i, i1, i2, i3, 0]
                        tmp = self.direct_kin(thetas)
                        if self.ist_rund(tmp) and self.direct_kin_wp(thetas)[2] > 0.1:
                            solutions.append(thetas)

                        i3 += (204 / 155.) * resolution
                    i2 += (297 / 155.) * resolution
                i1 += resolution
            percent += 50
        self.update_progresss(100)
        print "\n{a} Solutions found!".format(a=len(solutions))
        return solutions


    def find_best_solution(self, solutions):
        best_sol = np.array(solutions.pop(0))
        min = self.err_function(best_sol)
        for sol in solutions:
            if self.err_function(sol) < min:
                min = self.err_function(sol)
                best_sol = np.array(sol)

        tmp = np.degrees(best_sol)
        ##print "BestSolution: [  {0:5.2f} , {1:5.2f} , {2:5.2f} , {3:5.2f}  , {4:5.2f}  ]".format(tmp[0],tmp[1],tmp[2],tmp[3], tmp[4])
        print "BestSolution:", np.degrees(best_sol)
        print "Error:", min

        new_thetas = self.minimize(best_sol)
        print "optimized Solution: ", np.round(np.degrees(new_thetas), 2)
        print "optimized Error:", self.err_function(new_thetas)
        print "Point: ", self.direct_kin(new_thetas)
        print "z-Axes: ", self.direct_kin_wp(new_thetas)[2]

        return new_thetas


    def step_to_point(self, point):
        #print "Distance from arm: ", math.sqrt((point[0]+0.35)**2+(point[1])**2)
        if math.sqrt((point[0]+0.35)**2+(point[1])**2)>0.50:
            print "Point is not reachable"
            return None

        self.last_point = self.destination_point
        self.destination_point = point
        if self.last_solution is None:
            self.last_solution = self.find_best_solution(self.search_all_solutions(point, 6))

        erg = self.minimize(self.last_solution)



        if self.checkSolution(erg):
            return erg
        else:
            init_res=10
            erg=self.minimize(self.init_point)
            while not self.checkSolution(erg):
                if init_res<1:
                    print "Point is not reachable"
                    return None
                test = self.search_all_solutions(point, init_res)
                if len(test)>0:
                    erg = self.find_best_solution(test)
                    if not self.checkSolution(erg):
                        print "Point is not reachable"
                        return None


                init_res-=1

        self.last_solution=erg
        return erg


    def checkSolution(self, solution):

        if self.err_function(solution) > 0.0001 or not self.isSolutionValid(solution):
            return False
        else:
            return True

    def distance(point, last_solution):
        return 0

    def minimize(self, startPoint):
        erg = minimize(self.err_function, startPoint[0:4], method='SLSQP', options={'maxiter': 1e6, 'disp': False},
                       bounds=[(self.min_angles_[0], self.max_angles_[0]), (self.min_angles_[1], self.max_angles_[1]),
                               (self.min_angles_[2], self.max_angles_[2]), (self.min_angles_[3], self.max_angles_[3])])
        return [erg.x[0], erg.x[1], erg.x[2], erg.x[3], 0]


    def isSolutionValid(self, solution):
        if len(solution) != 5:
            return False
        for i in range(0, len(solution)):
            if math.isnan(solution[i]):
                return False
            elif solution[i] < self.min_angles_[i] or solution[i] > self.max_angles_[i]:
                return False
        if (self.direct_kin_wp(solution)[2] < 0.1):
            return False
        return True


    def isSolutionValid_single(self, solution, number):
        if solution < self.min_angles_[number] or solution > self.max_angles_[number]:
            return False

        return True


test = Kinematics()

# erg = test.step_to_point([0,0,0])


for i in xrange(0, 100, 1):
    erg = test.step_to_point([-0.05, 0.0 + i / 100., 0])
    # print erg
