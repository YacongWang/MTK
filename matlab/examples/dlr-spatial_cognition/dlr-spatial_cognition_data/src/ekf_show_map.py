#!/usr/bin/env python
# -*- coding: utf-8 -*-

from slam2D.mapdisplay import Map_Display, EKF_SLAM_Display
import cPickle
import sys

md = EKF_SLAM_Display(1024,768, center= (-160,0), scale_x= 0.08, scale_y=0.08)

mapfile ='ekf_slam.map'
if len(sys.argv) > 1:
    mapfile = sys.argv[1]
ekfmap_file = open(mapfile, 'r')
p = cPickle.Unpickler(ekfmap_file)
ekfmap = p.load()
md.map = ekfmap
md.set_savename(mapfile)
md.update()
md.wait_for_signal()
