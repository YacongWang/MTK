#!/usr/bin/env python
# -*- coding: utf-8 -*-

#
# call with python ekf_localizer.py ./dlr-spatial_cognition-e/dlr-spatial_cognition-e.txt
#




from numpy import *
from numpy.linalg import inv
#from scipy import linsolve, sparse

from dlr.importer import *
from dlr.modelfunctions import *
import cPickle

from slam2D.mapdisplay import Map_Display
from slam2D.helpers import *

import time
import math
import sys

class EKF_Localizer:
    def __init__(self, m):
        self.map = {}
        for i in m:
            if self.map.has_key( int(i[2]) ):
                print "already in map:", int(i[2])
            else:
                self.map[ int(i[2]) ] = i

    def ekf(self, mean_t1, cov_t1, odometry_t2, odometry_t2_cov, z_t2 ): # all type matrix or list of matrix
        self.prediction = mean_bar_t2 = f1( mean_t1.T.tolist()[0], odometry_t2)
        J1 = j1( mean_t1.T.tolist()[0], odometry_t2.T.tolist()[0] )
        J2 = j2( mean_t1.T.tolist()[0] )
        cov_bar_t2 = (J1 * cov_t1 * J1.T) + (J2 * odometry_t2_cov * J2.T) # J1*E*J1.T + J2*COV(u)*J2.T
        for z in z_t2:
            l = self.map[ z[2] ][0:3]
            l_cov = self.map[ z[2] ][3]
            z_bar = f3( mean_bar_t2.T.tolist()[0], l)
            J5 = j5( mean_bar_t2.T.tolist()[0], l )
            J5t = J5.T

            ### nach Probabilistic robotics
            S = J5 * cov_bar_t2 * J5t + l_cov
            K = cov_bar_t2 * J5t * inv(S)
            mean_bar_t2 += K * ( matrix( z[0:2] ).T - z_bar )
            #print "Chi-Square:",  ( matrix( z[0:2] ).T - z_bar ).T*inv(S)*( matrix( z[0:2] ).T - z_bar )
            cov_bar_t2 = ( eye(3) - K * J5)*cov_bar_t2
        return mean_bar_t2, cov_bar_t2

def main():
    f  = open('results/map.result', 'r')
    p = cPickle.Unpickler(f)
    m = p.load()
    md = Map_Display(1024,768, center= (-160,0), scale_x= 0.08, scale_y=0.08)
    md.map = m
    f.close()
    f  = open('results/pos.result', 'r')
    p = cPickle.Unpickler(f)
    pos = p.load()
    print pos[0]
    md.real_pos = pos
    md.robot = (0.0, 0.0, 0.0, zeros((3,3), dtype=float64))
    md.update()
    ekf = EKF_Localizer(m)
    if ( len(sys.argv) == 2 ):
        inputfile = sys.argv[1]
        data = read_test_data(inputfile, remove_unknown=True)

    mean = matrix( [ [0.0],[0.0],[0.0] ], dtype=float64)
    cov  = diag( (0.001, 0.001, 0.001) )
    for d in data:
        z = []
        zg = []
        for l in d.landmarks:
            z.append ( [l.x, l.y, l.id, l.cov] )
        mean, cov = ekf.ekf(mean, cov, d.step, d.step_cov, z)

        md.prediction = f1( md.prediction.T.tolist()[0], d.step )
        #md.prediction = ekf.prediction
        print "Prediction"
        print md.prediction

        md.robot = (mean[0,0], mean[1,0], mean[2,0], cov)
        md.info['Robot'] = md.robot[0:3]
        md.info['Odometry'] = d.step.T.tolist()[0]
        md.info['Num of z'] = len(d.landmarks)
        md.update()
        #md.wait_for_signal()
        time.sleep(0.08)
    md.wait_for_quit()

if __name__ == '__main__':
    main()
