#!/usr/bin/env python
# -*- coding: utf-8 -*-

#import psyco
#psyco.log()
#psyco.full()


from numpy import *
from numpy.linalg import inv
from scipy import linsolve, sparse
from pylab import *
from time import time
from math import cos, sin, pow, atan2
import re

from dlr.importer import *
from dlr.modelfunctions import *
from slam2D.helpers import mR2W, mW2R, stack_row, stack_diag
from slam2D.mapplot import *

import sys
import random
import copy

def build_jacobian(z, x_p, x_l, f_p, f_l):
    #print "Buld jacobian: "
    #print "Length: ", len(f_p), len(f_l)

    dim0 = len(z)
    dim1 = len(x_p) * 3 + len(x_l) * 2
    #print "Building Jacobi with Size:", dim0, dim1
    #J = zeros( (dim0, dim1), dtype=float64)
    J = sparse.lil_matrix( (dim0, dim1) )
    for row, f in enumerate(f_p):
        p0 = x_p[ f[1] ]
        p1 = x_p[ f[2] ]
        J3 = f[3](p0, p1)
        J4 = f[4](p0, p1)
        J_34 = hstack( (J3, J4)) #  das geht weil p0 und p1 immer nebeneinander liegen

        idx_p0 = f[1]
        idx_p1 = f[2]
        row_idx = row *3
        J[row_idx:row_idx+3, idx_p0*3:idx_p0*3+6] += J_34

    row_count = len(f_p)*3
    for f in f_l:
        # the landmarks
        p = x_p[ f[1] ]
        l = x_l[ f[2] ]
        # J5
        J5 = j5(p, l)
        # J6
        J6 = j6(p, l)
        idx0 = row_count
        idx1 = f[1]*3
        J[idx0:idx0+2, idx1:idx1+3] += J5
        idx1 = len(x_p)*3 + f[2]*2
        J[idx0:idx0+2, idx1:idx1+2] += J6
        row_count += 2
    return J

def chi_square_test(z_p, z_l, x_p,x_l, f_p, f_l):
    """
    """
    chi_square = 0.0
    for z,f in zip(z_p, f_p):
        # y_i - y(x)
        y = f[0]( x_p[ f[1] ], x_p[ f[2] ] )
        zi = matrix( [ z[0], z[1], z[2] ] ).T
        C = inv(z[3])
        chi_square += float((zi - y).T * C * (zi - y))
    for z,f in zip(z_l, f_l):
        y = f[0]( x_p[ f[1] ], x_l[ f[2] ] )
        zi = matrix( [ z[0], z[1] ] ).T
        C = inv( z[3] )
        chi_square += float((zi -y).T * C  * (zi - y))
    return chi_square

def build_zx(z_p, z_l):
    z_tmp = []
    for p in z_p:
        z_tmp += [ p[0], p[1], p[2] ]
    for l in z_l:
        z_tmp += [ l[0], l[1] ]
    return matrix( z_tmp ).T

def split_state(x_p, x_l, x):
    rxp = copy.copy(x_p)
    rxl = copy.copy(x_l)
    for i, _rxp in enumerate(rxp):
        _rxp[0] += x[i*3]
        _rxp[1] += x[i*3+1]
        _rxp[2] += x[i*3+2]

    offset = len(rxp) * 3
    for i, _rxl in enumerate(rxl):
        _rxl[0] += x[offset + i*2    ]
        _rxl[1] += x[offset + i*2 + 1]
    return rxp, rxl

class combined_f:
    __fp = []
    __fl = []
    __xp = []
    __xl = []

    def __init__(self, fp, fl):
        self.__fp = fp
        self.__fl = fl

    def __call__(self, xp_add_e, xl_add_e):
        fx = []
        for f in self.__fp:
            # call f store result in fx
            r = f[0]( xp_add_e [ f[1] ], xp_add_e[ f[2] ] )
            fx += r.T.tolist()[0]
        for f in self.__fl:
            # call f store result in fx
            r = f[0]( xp_add_e[ f[1] ], xl_add_e[ f[2] ] )
            fx += r.T.tolist()[0]
        return matrix(fx).T


def build_jacobian_numeric(z, x_p, x_l, f_p, f_l):
    eps = 0.000001
    f = combined_f(f_p,f_l)
    J = []
    for i in x_p:
        for j in range(3):
            i[j] += eps
            f1 = f(x_p, x_l)
            i[j] -= 2*eps
            f2 = f(x_p, x_l)
            i[j] += eps
            J.append( (f1 - f2) / (2*eps) )
    for i in x_l:
        for j in range(2):
            i[j] += eps
            f1 = f(x_p, x_l)
            i[j] -= 2*eps
            f2 = f(x_p, x_l)
            i[j] += eps
            J.append( (f1 - f2) / (2*eps) )
    jacobian = column_stack( J )
    return matrix(jacobian, dtype=float64)

def save_result(x_p, x_l):
    import cPickle
    ### x_l
    mapfile = 'map_'
    posfile = 'pos_'
    counter = 0
    while os.path.exists( mapfile+str(counter)+".result" ) or os.path.exists( posfile+str(counter)+".result" ):
        counter += 1
    map_file = open(mapfile+str(counter)+".result", 'w')
    pos_file = open(posfile+str(counter)+".result", 'w')
    if map_file:
        p = cPickle.Pickler(map_file)
        p.dump(x_l)
        map_file.close()
    if pos_file:
        p = cPickle.Pickler(pos_file)
        p.dump(x_p)
        pos_file.close()


def newton(z_p, z_l, x_p, x_l, f_p, f_l, C, iter):
    e = Easy_Plot()
    print "Newton..."
    z = build_zx(z_p, z_l)
    for i in range(iter):
        print "Step: ", i
        i+=1
        x = levenberg_iteration(z, z_p, z_l, x_p, x_l, f_p, f_l, C, 0.0)
        rxp, rxl = split_state(x_p, x_l, x)
        # update
        x_p = rxp
        x_l = rxl
    e.plot_full_map(x_p, x_l, 'cD', 'mD')
    save_result(x_p, x_l)
    show()

def levenberg(z_p, z_l, x_p, x_l, f_p, f_l, C):
    print "Levenberg..."
    e = Easy_Plot()
    # compute chi_square
    chi_square = chi_square_test( z_p, z_l, x_p, x_l, f_p, f_l)
    #print chi_square
    z = build_zx(z_p, z_l)
    lamb = 0.001
    i = 0
    chi_delta = 2.0
    while chi_delta > 0.0050 or chi_delta < 0.0:
        print "Step: ", i
        i+=1
        x = levenberg_iteration(z, z_p, z_l, x_p, x_l, f_p, f_l, C, lamb)
        rxp, rxl = split_state(x_p, x_l, x)
        #print "levenberg iteration", rxp
        n_chi = chi_square_test(z_p, z_l, rxp, rxl, f_p, f_l)
        chi_delta = (chi_square - n_chi)
        print "Chi square Test: ", n_chi, chi_square
        if ( n_chi >= chi_square ):
            lamb *= 10
        else:
            lamb *= 0.1
            # update
            x_p = rxp
            x_l = rxl
            chi_square = n_chi
            print "New iteration found"
    e.plot_full_map(x_p, x_l, 'cD', 'b+')
    save_result(x_p, x_l)
    show()
    return x_p

def levenberg_iteration(z, z_p, z_l, x_p, x_l, f_p, f_l, C, lamb):
    """
    """
    x = build_zx(x_p, x_l)
    J = build_jacobian(z, x_p, x_l, f_p, f_l)
    #J = build_jacobian_numeric(z, x_p,x_l,f_p,f_l)

    Jsp = J.tocsr()
    Csp = C.tocsr()
    alpha = Jsp.T.dot(Csp.dot(Jsp))

    fy_i = []
    for i, f in zip(z_p, f_p):
        l1 = matrix( [ i[0], i[1], i[2] ] ).T
        l2 = f[0]( x_p[ f[1] ], x_p[ f[2] ] )
        result = l1 -l2
        fy_i += result.T.tolist()[0]
    for i, f in zip(z_l, f_l):
        l1 = matrix( [ [ i[0] ], [ i[1] ] ] )
        l2 = f[0]( x_p[ f[1] ], x_l[ f[2] ] )
        result = l1 -l2
        fy_i += result.T.tolist()[0]
    fy_i = array ( fy_i, dtype=float64 ).T
    beta = Jsp.T.dot(Csp.dot(fy_i) )

    # Augmented alpha
    alpha.setdiag( sparse.extract_diagonal(alpha) * (1 + lamb) )

    #### sparse solver
    return linsolve.spsolve(alpha,beta)

def remove_empty_landmark_sets(data):
    data2 = []
    for i in data:
        if (len(i.landmarks) != 0):
            data2.append(i)
    return data2

def prepare_data(data, include_landmarks = True):
    z_p = [] # alle messungen Posen und Landmarken, Landmarken auf globale Koordinaten umgerechnet
    z_l = []
    x_p = [] # Initialer Zustandsvektor  (x < z )
    x_l = [] # Initialer Zustandsvektor  (x < z )
    dim = len(data) * 3  +3 # für die erste Covarianz p0 = (0,0,0)
    if ( include_landmarks ):
        for i in data:
            dim += len(i.landmarks) * 2 # Alle gemessenen Landmarken, 2 komponenten (x,y)
    #C = zeros( (dim,dim), dtype=float64 ) # Elementweise Invertierte Kovarianz Matrix
    C = sparse.lil_matrix( (dim,dim) )
    num_of_pose = 0
    num_of_marker = 0
    func_p = []
    func_l = []
    x_p.append( [0.0, 0.0, 0.0, matrix( zeros( ( 3,3), dtype=float64) ) ] )
    C[0:3, 0:3 ] = inv(diag( [0.0000000001, 0.0000000001, 0.0000000001] ) )
    z_p.append( [0,0,0, matrix( diag( (0.0000000001, 0.0000000001, 0.00000000001) )) ] )
    func_p.append( [f0, 0, 0, df0_dp0, df0_dp1] )
    for ci, d in enumerate(data):
        p0 = x_p[-1]
        #print p0
        R2W = mR2W( p0[2] )
        tmp = matrix( p0[0:3] ).T + R2W * d.step
        #print "TMP:", tmp
        x_p.append( tmp.T.tolist()[0] + [ d.step_cov]) # * diag( [1, 1.2, 0.7]) ])
        #print "Added: ", x_p[-1]
        idx = (ci+1) * 3
        C[ idx:idx+3, idx:idx+3 ] = inv(d.step_cov)
        func_p.append( [f2, ci, ci+1, j3, j4] )
        z_p.append( d.step.T.tolist()[0] + [d.step_cov] )

    if include_landmarks:
        c_row_count = 0
        ml = {} # map for landmarks (ID -> idx in z_l)
        for ci, d in enumerate(data):
            for cj, j in enumerate(d.landmarks):
                if ( not ml.has_key(j.id) ):
                    p = x_p[ci+1]
                    phi = p[2]
                    e = matrix( [ p[0], p[1], 0 ] ).T
                    f = mR2W(phi)
                    g = matrix( [ j.x, j.y, j.id ] ).T
                    x_l.append( ((e +  f * g).T.tolist()[0]) + [ j.cov ] )
                    ml[j.id] = len(x_l)-1
                idx = (len(x_p))*3 + c_row_count
                c_row_count += 2
                C[ idx:idx+2, idx:idx+2 ] = inv(j.cov)
                z_l.append( [j.x, j.y, j.id, j.cov] )
                # die korrespondierende Landmarke steht in der Map mv für diese Messung, dort ist der Index auf die Marke im state vector eingetragen
                ## add function reference
                func_l.append( [f3, ci+1, ml[j.id], j5, j6] )
        print "Row count: " , c_row_count
    return z_p, z_l, x_p, x_l, func_p, func_l, C

def load_data_and_prepare(inputfile, i):
    # get data from DLR File
    data = read_test_data(inputfile, remove_unknown=True)
    # Alle Datensätze ohne ID für Landmarken raus nehmen
    #if not include_landmarks:
    #    data = remove_empty_landmark_sets(data)
    return prepare_data(data, i) ## hopefully delete "data" here that saves some memory

def print_statistics(z_p, z_l, x_p, x_l, f_p, f_l, C):
    print "Statistics:"
    print "Number of Poses in State:      ", len(x_p)
    print "Number of Features in State:   ", len(x_l)
    print "Number of Pose Measurements:   ", len(z_p)
    print "Number of Feature Measurements:", len(z_l)
    chi_test = chi_square_test(z_p, z_l, x_p, x_l, f_p, f_l)
    print "Chi Square Test:            ", chi_test
    estimated_chi = (3*len(z_p) + 2* len(z_l)) - ( 3*len(x_p)+2*len(x_l))
    print "Estimated Chi Square: ", estimated_chi
    print "Covariance Calibration Factor: ", chi_test / estimated_chi
    print "Sigma Calibration Factor: ", sqrt(chi_test / estimated_chi )

def main():
    ## Global Setting
    include_landmarks = True
    #""" Berechne mit Levenberg Marquardt (beliebige) Minimierungs-Probleme """
    # Settings for matrix output
    #set_printoptions(threshold=nan, linewidth=nan, precision=3, suppress=True)
    print isinteractive()
    # extract command line parameters
    inputfile = 'circle_output_w_id'
    if ( len(sys.argv) == 2 ):
        inputfile = sys.argv[1]
        z_p, z_l, x_p, x_l, func_p, func_l, C = load_data_and_prepare(inputfile, include_landmarks)
        levenberg(z_p, z_l, x_p, x_l, func_p, func_l, C)
        print_statistics(z_p, z_l, x_p, x_l, func_p, func_l, C)
    elif ( len(sys.argv) > 2 ):
        inputfile = sys.argv[-1]
        z_p, z_l, x_p, x_l, func_p, func_l, C = load_data_and_prepare(inputfile, include_landmarks)
        m = re.compile("(?<=newton:)\w+").search(sys.argv[1], 1)
        if ( m ):
            newton(z_p, z_l, x_p, x_l, func_p, func_l, C, int(m.group(0)))
        else:
            newton(z_p, z_l, x_p, x_l, func_p, func_l, C, 15)
        print_statistics(z_p, z_l, x_p, x_l, func_p, func_l, C)

if __name__ == "__main__":
    main()

