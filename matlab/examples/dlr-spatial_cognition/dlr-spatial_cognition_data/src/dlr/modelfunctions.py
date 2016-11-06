#!/usr/bin/env python
 # -*- coding: utf-8 -*-
 
from numpy import *
from math import sin, cos

## model functions
def f1(p, d):
    phi = p[2]
    rot = matrix( [ [ cos(phi), -sin(phi), 0 ],
                    [ sin(phi), cos(phi), 0 ],
                    [ 0, 0, 1 ] ])
    return matrix( [ [p[0]],[p[1]], [p[2]] ] ) + rot * d


def f2(p0, p1):
    phi = p0[2]
    r = matrix( [ [ cos(phi), sin(phi), 0],
                  [-sin(phi), cos(phi), 0],
                  [0,0,1] ] )

    d = matrix( [ [p1[0] - p0[0]],
                  [p1[1] - p0[1]],
                  [p1[2] - p0[2]] ] )
    return r * d

def f3(p, l):
    #print "f3(",l,p,")"
    phi = p[2]
    r = matrix( [ [  cos(phi), sin(phi) ],
                  [ -sin(phi), cos(phi) ]] )
    d = matrix( [ [ l[0] - p[0] ],
                  [ l[1] - p[1] ] ] )
    return r * d

def f4(p, m):
    phi = p[2]
    robot2world = matrix( [ [ cos(phi), -sin(phi)],
                            [ sin(phi), cos(phi)] ])
    pos = matrix( p[0:2] ).T
    return pos + robot2world * matrix ( m ).T

def df4_dp(p, m):
    phi = p[2]
    return matrix( [ [1, 0, -sin(phi)*m[0] - cos(phi)*m[1] ],
                     [0, 1,  cos(phi)*m[0] - sin(phi)*m[1] ] ])

def df4_dm(p, m):
    phi = p[2]
    return matrix( [ [cos(phi), -sin(phi) ],
                     [sin(phi), cos(phi) ] ])

def j1(p0, d):
    phi = p0[2]
    return matrix( [ [1, 0, -sin(phi) * d[0] - cos(phi) * d[1]],
                    [0, 1,  cos(phi) * d[0] - sin(phi) * d[1]],
                    [0, 0, 1] ])

def j2(p0):
    phi = p0[2]
    return matrix( [ [cos(phi), -sin(phi), 0],
                     [sin(phi),  cos(phi), 0],
                     [0,           0,      1] ])


def j3(p0, p1):
    phi = p0[2]
    return matrix( [ [ -cos(phi), -sin(phi), -sin(phi)*( p1[0] - p0[0]) + cos(phi)*( p1[1] - p0[1] ) ],
                     [  sin(phi), -cos(phi), -cos(phi)*( p1[0] - p0[0]) - sin(phi)*( p1[1] - p0[1] ) ],
                     [ 0, 0, -1 ]
                     ])


def j4(p0, p1):
    phi = p0[2]
    return matrix( [ [  cos(phi), sin(phi), 0 ],
                    [ -sin(phi), cos(phi), 0 ],
                    [ 0, 0, 1 ]
                    ])

def j5(p, l):
    phi = p[2]
    return matrix( [ [ -cos(phi), -sin(phi), -sin(phi)*( l[0] - p[0]) + cos(phi)*( l[1] - p[1] ) ],
                    [  sin(phi), -cos(phi), -cos(phi)*( l[0] - p[0]) - sin(phi)*( l[1] - p[1] ) ] ])


def j6(p,l):
    phi = p[2]
    return matrix( [ [ cos(phi), sin(phi) ],
                    [ -sin(phi), cos(phi) ]
                    ])

def f0(p0,p1):
    return matrix( [ p0[0], p0[1], p0[2] ] ).T

def df0_dp0(p0,p1):
    return matrix( eye( 3, dtype=float64))

def df0_dp1(p0,p1):
    return matrix( zeros( (3,3), dtype=float64))



if __name__ == '__main__':
    p = [ 1.2, 3.4, 0.1 ]
    m = [12.0, 34.2 ]

    l = f3(p, m)
    print f4( p, l.T.tolist()[0]).T.tolist()[0]
    print m
    # okay f3 und f4 drehen hin und wieder zurück
