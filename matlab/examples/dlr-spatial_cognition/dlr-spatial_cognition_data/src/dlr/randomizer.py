#!/usr/bin/env python
# -*- coding: utf-8 -*-

import importer
import sys
import numpy as nm
import random
import os.path
from scipy import matrix
from scipy.linalg import *

def random_gauss(covariance):
    # test for symmetry and positive semi-definite
    # .... :-)
    # cholesky decomposition
    #print matrix(covariance)
    L = cholesky( covariance, lower = 1 )
    x = []
    for i in range(0, covariance.shape[0]):
        x.append( random.gauss(0,1) )
    x = matrix(x).T
    return L * x

def randomize_steps(data, alpha):
    for i in data:
        g1 = random_gauss( i.step_cov )
        i.step = i.step + alpha * g1
        i.step_cov = (1+alpha) * i.step_cov


def randomize_circles(data, alpha):
    for i in data:
        for j in i.landmarks:
            g = random_gauss(j.cov)
            r = matrix( [ [j.x],[j.y] ] ) + alpha * g
            j.x = float(r[0])
            j.y = float(r[1])
            j.cov = (1+ (alpha * alpha) ) * j.cov


def randomize_lines(data, alpha):
    for i in data:
        for j in i.landmarks:
            # build small matrix with sigma_alpha (or is the noise only 1D?)
            g = random.gauss(0, j.sigma_alpha)
            #print "Randomizer: (alpha, new_alpha)", j.alpha, j.alpha + alpha * g
            j.alpha += alpha * g
            j.xtop += alpha* g
            j.xbottom += alpha* g
            j.sigma_alpha = (1+ alpha * alpha) * j.sigma_alpha

def print_help():
    print """Usage: randomizer.py map_file [alpha = 0.3]

input_file: input data set
alpha:      noise factor"""
    sys.exit(0)

def main():
    inputfile = 'map.cirlces'
    alpha1 = alpha2 = 0.3

    if len(sys.argv) == 1:
        print_help()
    if ( len(sys.argv) > 1 ):
        inputfile = sys.argv[1]
    if ( len(sys.argv) > 2 ):
        alpha1 = alpha2 = float(sys.argv[2])

    data = importer.read_test_data(inputfile)

    if isinstance(data[0].landmarks[0], importer.Circular_Landmark):
        save_appendix = 'circles'
        randomize_circles(data, alpha2)
    else:
        save_appendix = 'lines'
        randomize_lines(data, alpha2)

    ## TODO: read from commandline
    rand_steps = True
    if ( rand_steps ):
        randomize_steps(data, alpha1)

    save_path = os.path.dirname(inputfile)
    save_file = os.path.join(save_path, 'map_distorted.'+save_appendix)
    importer.save_data(data, save_file)

if __name__ == "__main__":
    main()
