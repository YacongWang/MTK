#!/usr/bin/env python 
# -*- coding: utf-8 -*-


# file: export.py
# Example file for exporting the results of the Levenberg-Marquardt optimizer
# (pos.result , map.result from the results directory)
# The file are pickled Python objects. (list of vectors)
# this small exmaple script read these objects and print them in a CSV-style
# format.


# first import the pickle library
import cPickle

# import the sys module to get access to commandline parameter
import sys

# take the first command line parameter as result file
f = open(sys.argv[1])

# make pickle object
p = cPickle.Unpickler(f)

# load the data structure into res
# res then contains a list with the object
# the position is a 3-dim vector (x,y,angle) and a corresponding 3x3 matrix
res = p.load()

# print the objects in a desirable format
for i in res:
    b = []
    for k in i[3].flat:
            b.append(k)
    
    print '%f ; %f ; %f ; %f ; %f ; %f ; %f ; %f ; %f ; %f ; %f ; %f' % ( i[0], i[1], i[2],
            b[0], b[1], b[2],
            b[3], b[4], b[5],
            b[6], b[7], b[8])

