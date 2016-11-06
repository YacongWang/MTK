#! /usr/bin/env python
# -*- coding: utf-8 -*-

from numpy import matrix
from numpy import zeros
from numpy.linalg import inv

import copy

class CamError(Exception):
    def __init__(self, value = ''):
        self.value = value

    def __str__(self):
        return repr(self.value)


class Camera:
    width = 0.0
    height = 0.0
    scale_x = 0.0
    scale_y = 0.0
    offset_x = 0.0
    offset_y = 0.0
    distorted= False
    mount_index = 0

    sensor2tcp = matrix([])
    sensor2world = matrix([])

    def __init__(self):
        pass

    def distort(self, x, y):
        return (x,y)

    def normalized2image(self, x, y):
        x,y = self.distort(x,y)
        rx = x* self.scale_x + self.offset_x
        ry = y* self.scale_y + self.offset_y
        return (rx, ry)

    def inv(self, m):
        r = zeros( (3,4) )
        r[0:3, 0:3] = m[0:3, 0:3].T
        r[0:3, 3:4] = (-1 * (m[0:3, 0:3].T)) * m[0:3, 3:4]
        return r


    def world2sensor(self, v):
        # sensor2world invertieren
        s2w_inv = self.inv(self.sensor2world)
        v = [v[0], v[1], v[2], 1]
        r = s2w_inv * matrix( v ).T
        return ( float(r[0]), float(r[1]), float(r[2]) )

    def sensor2image(self, v):
        x = 0.0
        y = 0.0
        if ( v[2] >= 0):
            x = v[0] / v[2];
            y = v[1] / v[2];
            if self.distorted:
                x, y = self.distort (x, y)
            x = self.scale_x * x + self.offset_x
            y = self.scale_y * y + self.offset_y
        else:
            x = y = float('nan')
        return (x,y)

    def world2image(self, v):
        if len(v) == 2:
            v = (v[0], v[1], 0 )
        r = self.sensor2image ( self.world2sensor(v) );
        return (float(r[0]), float(r[1]) )


    def read_and_split(self, camfile):
        f = open(camfile, 'r')
        data = f.read()
        return data.split(' ')

    def next_token(self):
        if ( self.token_idx < len(self.file_content)):
            token = self.file_content[self.token_idx]
            self.token_idx += 1
            return token
        else:
            return ''

    def check(self, c):
        t = self.next_token()
        if ( t == c ):
            return
        else:
            print "Expecting ", c, "got", t
            raise CamError()

    def scan_matrix(self):
        try:
            # reading 4x3 Matrix
            m = []
            for i in range(3):
                self.check('[')
                tmp = []
                for i in range(4):
                    token = self.next_token()
                    tmp.append( float(token.rstrip(',')) )
                m.append(copy.copy(tmp))
                tmp = []
                self.check(']')
            return matrix( m )
        except CamError:
            print "Could not Parse"

    def read_sensor2Tcp(self):
        print "Sensor2TCP"
        self.sensor2tcp = self.scan_matrix()
        print self.sensor2tcp

    def read_sensor2World(self):
        print "Sensor2World"
        self.sensor2world = self.scan_matrix()
        print self.sensor2world

    def read_width(self):
        self.width = int(self.next_token())
        print "width:" , self.width

    def read_height(self):
        self.height = int(self.next_token())
        print "height:" , self.height

    def read_scaleX(self):
        self.scale_x = float(self.next_token())
        print "scale_x:" , self.scale_x


    def read_scaleY(self):
        self.scale_y = float(self.next_token())
        print "scale_y:" , self.scale_y

    def read_offsetX(self):
        self.offset_x = float(self.next_token())
        print "offset_x:" , self.offset_x

    def read_offsetY(self):
        self.offset_y = float(self.next_token())
        print "offset_y:" , self.offset_y

    def read_mountIndex(self):
        self.mount_index = int(self.next_token())
        print "mount_index:", self.mount_index

    def check_format(self):
        token = self.next_token()
        print "Format: ", token[2:]
        if ( self.__class__.__name__ != token[2:]):
            raise CamError("Wrong Format: "+token)

    def parse_cam_file(self):
        print "Parse"
        self.file_content = self.f.read().split()
        self.token_idx = 0
        token = ''
        try:
            self.check('{')
            self.check_format()
            token = self.next_token()
        except CamError:
            print CamError()
        while token != '':
            callf = str('read_')+token
            callf = callf.rstrip(':')
            #print "Next Token: ", callf
            if hasattr(self, callf):
                member = getattr(self, callf)
                member()
            else:
                print "Unknown Token:", token
            token = self.next_token()

    def load(self, camfile):
        self.f = open(camfile, 'r')
        self.parse_cam_file()

if __name__ == "__main__":
    ### do some tests here
    pass
