#!/usr/bin/env python
# -*- coding: utf-8 -*-

import dlr.display
import dlr.importer
import sys
import os.path
from dlr.sensormodel.RadialDistortedCamera import RadialDistortedCamera
from dlr.sensormodel.Camera import CamError

from PyQt4 import Qt,QtGui, QtCore


def print_help():
    print """Usage: data_viewer processed_data.(circles|lines) camera_model.cam image_path"""


if __name__ == "__main__":
    image_path = 'images/'
    data_path = '.'
    data_basename = ''
    inputfile = 'map.circles'
    if ( len(sys.argv) < 2 ):
        print_help()
        sys.exit(0)
    elif ( len(sys.argv) == 2 ):
        data_info = dlr.importer.Data_Loader(sys.argv[1])

    rcam = RadialDistortedCamera()
    try:
        rcam.load(data_info.camfile)
    except CamError, c:
        print c.value

    ## GUI Setup
    app = QtGui.QApplication(sys.argv)
    image_display = dlr.display.Image_Display()
    image_display.set_camera(rcam)
    image_display.set_image_path(data_info.image_path)
    image_display.set_data(data_info.data)
    image_display.show()
    sys.exit(app.exec_())
