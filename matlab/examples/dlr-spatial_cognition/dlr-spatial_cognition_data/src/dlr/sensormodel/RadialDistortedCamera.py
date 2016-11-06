#! /usr/bin/env python
# -*- coding: utf-8 -*-

from math import sqrt
import Camera
import mathhelpers

class RadialDistortedCamera(Camera.Camera):
    a2 = 0.0
    a3 = 0.0
    b1 = 0.0
    b2 = 0.0
    b3 = 0.0
    r_max = 0.0
    r_primemax = 0.0


    def __init__(self):
        pass

    def __str__(self):
        return "a2: "+str(self.a2)+"\na3: "+str(self.a3)+"\nb1: "+str(self.b1)+"\nb2: "+str(self.b2)+"\nb3: "+str(self.b3)+"\nr_max: "+str(self.r_max)+"\nr_primemax: "+str(self.r_primemax)

    def set_distortion(self, a2, a3, b1, b2, b3, r_max=0.0):
        self.a2 = a2
        self.a3 = a3
        self.b1 = b1
        self.b2 = b2
        self.b3 = b3
        self.distorted = (a2!=0 or a3!=0 or b1!=0 or b2!=0 or b3!=0)
        if not self.distorted:
            self.r_max = 10.0;
        elif r_max > 0:
            self.r_max = r_max
        else:
            p = []
            p.append(1)
            p.append( 2*a2 + 2*b1)
            p.append( 3*a3 + a2*b1 + b1*b1 + 2*b2)
            p.append( 2*a3*b1 + 2*b1*b2 + 2*b3)
            p.append( a3*b2 + b2*b2 - a2*b3 + 2*b1*b3)
            p.append( 2*b2*b3)
            p.append( b3*b3)
            r = 0.0
            l = 1.0
            while ( r < 5.0 and l > 0.01 ):
                l = mathhelpers.distance_to_polynominal_root(p, 6, r )
                r += l
            self.r_max = r
        self.r_primemax = self.r_prime(self.r_max)

    def r_prime(self, r):
        if (r < 0 or r > self.r_max):
            return float('nan')
        nom = r*(self.a2+self.a3*r)
        denom = 1+r*(self.b1+r*(self.b2+r*self.b3))
        return r*(1+nom/denom)


    # Radial distort has coef
    def read_coef(self):
        try:
            self.check('{')
            a2 = float(self.next_token())
            a3 = float(self.next_token())
            b1 = float(self.next_token())
            b2 = float(self.next_token())
            b3 = float(self.next_token())
            self.set_distortion(a2,a3,b1,b2,b3)
            self.check('}')
        except camera.CamError:
            print "could not parse"


    def distort(self, x, y):
        r = sqrt( x*x + y*y )
        if (r > self.r_max):
            return ( float('nan'), float('nan') )
        nom = r*(self.a2 + self.a3 * r)
        denom = 1+r * (self.b1 + r*(self.b2+r*self.b3))
        f = 1+nom/denom
        return (x*f, y*f)

if __name__ == "__main__":
    c = RadialDistortedCamera()
    try:
        c.load("test.cam")
        print c
    except camera.CamError:
        print "Wrong Format"
    print c.distort(0.4, 0.5)
    print c.world2image( (0.25453, -3.36067) )
