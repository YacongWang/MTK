#!/usr/bin/env python
# -*- coding: utf-8 -*-

import importer
import sys
import numpy as nm
import random
import os.path
import math
from scipy import matrix
from scipy.linalg import *

def change_alpha_lines(data, correction_term):
    for i in data:
        for j in i.landmarks:
            # build small matrix with sigma_alpha (or is the noise only 1D?)
            #print "Randomizer: (alpha, new_alpha)", j.alpha, j.alpha + alpha * g
            print "adding: ", correction_term, " rad"
            j.alpha += correction_term

def print_help():
    print """Usage: errorterminator.py map_file [alpha = -90]

input_file: input data set
alpha:      noise factor"""
    sys.exit(0)

def main():
    inputfile = 'map.lines'
    alpha = -90
    if len(sys.argv) == 1:
        print_help()
    if ( len(sys.argv) > 1 ):
        inputfile = sys.argv[1]
    if ( len(sys.argv) > 2 ):
        alpha = float(sys.argv[2])

    data = importer.read_test_data(inputfile)
    save_appendix = 'lines'
    change_alpha_lines(data, math.radians(alpha))

    save_path = os.path.dirname(inputfile)
    save_file = os.path.join(save_path, 'map_corrected.'+save_appendix)
    importer.save_data(data, save_file)

if __name__ == "__main__":
    main()
