#!/usr/bin/env python
# -*- coding: utf-8 -*-

from numpy import *
from math import cos,sin

## Helper (Rotationsmatrizen erzeugen)
def mR2W (phi, rank = 3):
    if ( rank == 3 ):
        return matrix( [ [cos(phi), -sin(phi), 0], [sin(phi), cos(phi), 0], [0,0,1] ] )
    if ( rank == 2 ):
        return matrix( [ [cos(phi), -sin(phi)], [sin(phi), cos(phi)] ] )

def mW2R (phi, rank =3):
    if ( rank == 3 ):
        return matrix( [ [cos(phi), sin(phi), 0], [-sin(phi), cos(phi), 0], [0,0,1] ] )
    if ( rank == 2 ):
        return matrix( [ [cos(phi), sin(phi)], [-sin(phi), cos(phi)] ] )


def stack_diag(a, d):
    return vstack( (hstack( (a, zeros( (a.shape[0], d.shape[1])) ) ), hstack( (zeros( (d.shape[0], a.shape[1]) ), d ))))

def stack_row(J, J1, J2, idx1, idx2, row):
    if J1.shape[0] != J2.shape[0]:
        raise Exception("Jacobian must have same row size")
    J_row = zeros( (J1.shape[0], J.shape[1]), dtype=float64)
    J_row[0:J1.shape[0], idx1:idx1+J1.shape[1] ] += J1
    J_row[0:J2.shape[0], idx2:idx2+J2.shape[1] ] += J2
    return vstack( ( J, J_row ) )
