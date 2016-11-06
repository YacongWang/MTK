#! /usr/bin/env python
# -*- coding: utf-8 -*-

import os.path
import numpy


class Landmark:
    def __init__(self):
        self.id = []
        self.quality = 0.0
        self.cov = numpy.matrix([])

class Circular_Landmark(Landmark):
    def __init__(self):
        Landmark.__init__(self)
        self.id = -1
        self.x = 0.0
        self.y = 0.0

    def __str__(self):
        return "LANDMARK_C %.8f %.8f %.8f %.8f %.8f %.8f %d\n" % (self.x, self.y, self.quality, float(self.cov[0,0]), float(self.cov[0,1]), float(self.cov[1,1]), self.id)

class Line_Landmark(Landmark):
    def __init__(self):
        Landmark.__init__(self)
        self.xtop = 0.0
        self.ytop = 0.0
        self.xbottom = 0.0
        self.ybottom = 0.0
        self.alpha = 0.0
        self.beta = 0.0
        self.sigma_alpha = 0.0

    def __str__(self):
        s = "LANDMARK_L %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f" % (self.xtop, self.ytop, self.xbottom,
                                                                    self.ybottom, self.alpha, self.beta,
                                                                    self.quality, self.sigma_alpha)
        for i in self.id:
            s +=" %d" % int(i)
        s+="\n"
        return s

class Discrete_Step:
    def __init__(self):
        self.image = ''
        self.step = numpy.matrix([], dtype=numpy.float64)
        self.step_cov = numpy.matrix([], dtype=numpy.float64)
        self.landmarks = []

    def __str__(self):
        return "STEP %s %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f\n" % (self.image, float(self.step[0]), float(self.step[1]), float(self.step[2]), float(self.step_cov[0,0]), float(self.step_cov[0,1]), float(self.step_cov[1,1]), float(self.step_cov[0,2]), float(self.step_cov[1,2]), float(self.step_cov[2,2]))


class Data_Loader:
    def __init__(self, inputfile, remove_unknown=False, camfile_suffix='cam', image_suffix='jpg', image_dir='images'):
        self.data = read_test_data(inputfile, remove_unknown)
        self.inputfile = inputfile
        self.data_path = os.path.dirname(self.inputfile)
        self.data_basename = os.path.basename(self.data_path)
        self.camfile = os.path.join(self.data_path, self.data_basename+"."+camfile_suffix)
        self.image_path = os.path.join(self.data_path, image_dir)
        self.image_suffix = "."+image_suffix

    def get_image(self, step):
        return os.path.join(self.image_path, self.data[step].image, self.image_suffix)

    def get_image(self, image_name):
        return os.path.join(self.image_path, image_name, self.image_suffix)

def read_test_data(inputfile, remove_unknown=False):
    d = []
    new_step = False
    f = open(inputfile, 'r')
    lines = f.readlines()
    for i in lines:
        i = i.strip()
        if ( len(i) == 0 or i[0] == '#' ): # remove comments and blank lines
            continue
        items = i.split()
        if ( items[0] == "STEP" ):
            tmp = Discrete_Step()
            tmp.image = items[1]
            cXX = numpy.double(items[5])
            cXY = numpy.double(items[6])
            cYY = numpy.double(items[7])
            cXPhi = numpy.double(items[8])
            cYPhi = numpy.double(items[9])
            cPhiPhi = numpy.double(items[10])
            tmp.step = numpy.matrix([ float(items[2]), float(items[3]), float(items[4]) ], dtype=numpy.float64 ).T
            tmp.step_cov = numpy.matrix([ [cXX, cXY, cXPhi], [cXY, cYY, cYPhi], [cXPhi, cYPhi, cPhiPhi] ] ) # <cXX> <cXY> <cYY> <cXPhi> <cYPhi> <cPhiPhi>
            d.append( tmp )
        if ( items[0] == "LANDMARK_C" ):
            # circles
            l = Circular_Landmark()
            l.x = float(items[1])
            l.y = float(items[2])
            l.quality = float(items[3])
            # we have two formats with landmark id and without
            #l.id = []
            #try:
            #    l.id.append( int(items[7]) )
            #except:
            #    l.id.append(-1)
            try:
                l.id =int(items[7])
            except:
                l.id = -1
            cxx = float(items[4]); cxy = float(items[5]); cyy = float(items[6])
            l.cov = numpy.matrix([ [cxx, cxy],[cxy,cyy] ] )
            if not ( l.id == -1 and remove_unknown ):
                d[-1].landmarks.append(l)
        if ( items[0] == "LANDMARK_L" ):
            l = Line_Landmark()
            l.xtop = float(items[1])
            l.ytop = float(items[2])
            l.xbottom = float(items[3])
            l.ybottom = float(items[4])
            l.alpha = float(items[5])
            l.beta = float(items[6])
            l.quality = float(items[7])
            l.sigma_alpha = float(items[8])
            l.id = []
            for i in range(9, len(items)):
                try:
                    l.id.append( int(items[i] ) )
                except:
                    print "Read error.... trying to continue"
            d[-1].landmarks.append(l)
    f.close()
    return d


def write_preamble(f, what=0):
    if what == 0: #### CIRCLES
        f.write("""# This file contains metric planar odometry and landmark observations with covariance.
#  It is derived from the original odometry / image dataset by a circular landmark detection 
#  algorithm. The original dataset as well as calibration and landmark detection code is
#  available upon request.
# 
#  Format:
#  The robot coordinate system is located in the robot's center with -Y pointing forward and X pointing right.
#  STEP <image.ppm> <dX> <dY> <dPhi> <cXX> <cXY> <cYY> <cXPhi> <cYPhi> <cPhiPhi>
#  A single step of movement with odometry information.
#  <image.ppm> is the filename of the image
#  taken AFTER the odometry step. <dX>, <dY>, <dPhi> is the location of the robot after movement
#  in the coordinate system of the robot before moving. Units are meter and radian.
#  {{cXX,cXY,cXPhi},{cXY,cYY,cYPHi},{cXPhi,cYPhi,cPhiPhi}} is the covariance matrix of {dX,dY,dPhi}
#  with corresponding units (meter^2, meter*radian, radian^2).
#  The image is not needed, since the result of the landmark detection is provided but it is included for
#  reference.
# 
#  LANDMARK <pX> <pY> <quality> <cXX> <cXY> <cYY> <ID>
#  if a Landmark has multiple IDs the IDs are just added to the end:#  LANDMARK <pX> <pY> <quality> <cXX> <cXY> <cYY> <ID0> <ID1> ... <IDn>
#  Observation of a landmark in the image taken after the previous odometry step.
#  {pX, pY} is the position in robot coordinates (unit meter).
#  {{cXX, cXY}, {cXY, cYY}} is the covariance of {pX, pY}
#  <quality> ranging from 0 (bad) to 1 (good) indicates the confidence of the computer
#  vision landmark detector.

""")
    else:
        f.write("""# This file contains metric planar odometry and landmark observations with id.
#  It is derived from the original odometry / image dataset by a vertical lines detection
#  Format:
#  The robot coordinate system is located in the robot's center with -Y pointing forward and X pointing right.
#  STEP <image.ppm> <dX> <dY> <dPhi> <cXX> <cXY> <cYY> <cXPhi> <cYPhi> <cPhiPhi>
#  A single step of movement with odometry information.
#  <image.ppm> is the filename of the image
#  taken AFTER the odometry step. <dX>, <dY>, <dPhi> is the location of the robot after movement
#  in the coordinate system of the robot before moving. Units are meter and radian.
#  {{cXX,cXY,cXPhi},{cXY,cYY,cYPHi},{cXPhi,cYPhi,cPhiPhi}} is the covariance matrix of {dX,dY,dPhi}
#  with corresponding units (meter^2, meter*radian, radian^2).
#  The image is not needed, since the result of the landmark detection is provided but it is included for
#  reference.
#
#  LANDMARK <xntop> <yntop> <xnbottom> <ynbottom> <alpha> <beta> <quality> <sigma_alpha> <ID> [<ID> ... ]
#  Observation of a landmark in the image taken after the previous odometry step.
#  {xntop, yntop, xnbottom, ynbottom} are just for visualization and are no help for data assoziation! (Don't cheat!)
#  {alpha, beta} is the angle of the observation in robot coordinates (unit ?).
#  <quality> ranging from 0 (bad) to 1 (good) indicates the confidence of the computer
#  vision landmark detector.
 #  <ID> is the unique ID of the Landmark
#  if there are multiple IDs for a Landmark the line is simply longer and the rest contains so additional landmarks

""")

def save_data(data, name="output.data"):
    if len(data) == 0:
        return
    out = open(name, 'w')
    data_type = 0  # CIRCLES
    ## do we have landmarks?
    if len(data[0].landmarks) > 0:
        if isinstance( data[0].landmarks[0], Line_Landmark):
            data_type = 1
    write_preamble(out, data_type)
    for d in data:
        out.write( str(d) )
        for l in d.landmarks:
            out.write( str(l) )
    out.close()

if __name__ == "__main__":
    data = read_test_data("map.circles")
    print "Steps:", len(data)
    sum = 0
    for i in data:
        #print i
        sum += len(i.landmarks)
    print "Num. of Landmarksmeasurements: ", sum
    print "Images: " , data[0].image
    for l in data[0].landmarks:
        print l

    print "======== Okay the same with Lines =========="
    data2 = read_test_data("map.lines")
    print "Steps:", len(data)
    sum = 0
    for i in data2:
        sum += len(i.landmarks)
    print "Num. of Landmarksmeasurements: ", sum
    print "Images: " , data2[0].image
    for l in data2[0].landmarks:
        print l

