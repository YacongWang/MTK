#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
from PyQt4 import Qt,QtGui, QtCore
import os.path
from importer import *
import math
from numpy import arange

class Image_Display(QtGui.QWidget):
    current_step = 0
    current_image = QtGui.QImage()
    data = []
    image_path = '.'
    camera = None
    data_type = 0 # 0 means circles
                  # 1 means lines

    def __init__(self, parent = None):
        QtGui.QWidget.__init__(self, parent)
        self.resize(768,400)

    def set_data(self, data):
        self.data = data
        for i in self.data:
            if len(i.landmarks) > 0:
                if isinstance(i.landmarks[0], Line_Landmark):
                    self.data_type = 1
                    break
        self.load_image()

    def set_image_path(self, ip):
        print ip
        self.image_path = ip

    def set_camera(self, cam):
        self.camera = cam

    def load_image(self):
        i = os.path.join(self.image_path, self.data[self.current_step].image+".jpg")
        #print "Load image", i
        self.current_image = QtGui.QImage(i)

    def next(self, i):
        self.current_step += i
        if self.current_step < 0:
            self.current_step = 0
        if self.current_step >= len(self.data):
            self.current_step = len(self.data)-1
        self.load_image()

    def on_show(self):
        self.repaint()

    def back(self, i):
        self.current_step -= i
        if self.current_step < 0:
            self.current_step = 0
        if self.current_step >= len(self.data):
            self.current_step = len(self.data)-1
        self.load_image()


    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_Q:
            sys.exit(0)
        elif event.key() == QtCore.Qt.Key_Left:
            self.back(1)
        elif event.key() == QtCore.Qt.Key_Right:
            self.next(1)
        elif event.key() == QtCore.Qt.Key_Up:
            self.next(10)
        elif event.key() == QtCore.Qt.Key_Down:
            self.back(10)
        elif event.key() == QtCore.Qt.Key_Space:
            print "Saving..."
            while ( self.current_step != len(self.data)-1 ):
                self.next(1)
                self.repaint()
                self.save_framebuffer()
                print self.current_step
            print "done"
        self.repaint()


    def draw_cross(self, x, y, context):
        lines = [];
        lines.append( QtCore.QLine(x-6, y-6, x+6, y+6) )
        lines.append( QtCore.QLine(x-6, y+6, x+6, y-6) )
        context.drawLines(lines)

    def draw_circles(self, context):
        context.drawText(QtCore.QPoint(1, 300), "Step: "+str(self.current_step))
        context.save()
        context.setPen(QtCore.Qt.yellow)
        l = self.data[self.current_step].landmarks
        if len(l) > 0:
            for i in l:
                x,y = self.camera.world2image( (i.x, i.y) )
                self.draw_cross(x, y, context)
                context.drawText(x+5, y+15, str(i.id))
        context.restore()

    def draw_lines(self, context):
        context.drawText(QtCore.QPoint(1, 300), "Step: "+str(self.current_step))
        context.save()
        context.setPen(QtCore.Qt.yellow)
        l = self.data[self.current_step].landmarks
        if len(l) > 0:
            for i in l:
                first = True
                ox = 0.0
                oy = 0.0
                for fC in arange(0, 1.01, 0.1):
                    x,y = self.camera.normalized2image( fC * i.xtop+(1-fC)*i.xbottom, fC*i.ytop+(1-fC)*i.ybottom  )
                    if not first:
                        context.drawLine(x,y,ox,oy)
                    else:
                        # Sigma as horizontal bar
                        context.setPen(QtCore.Qt.red)
                        sigma_in_pixel = self.camera.scale_x * i.sigma_alpha
                        context.drawLine(x-sigma_in_pixel, y, x+sigma_in_pixel, y)
                        context.setPen(QtCore.Qt.yellow)
                        first = False

                    ox = x
                    oy = y
                context.drawText(x+5, y+15, str(i.id))
        context.restore()

    def draw_info(self, context):
        l = self.data[self.current_step].landmarks
        yoffset = 0
        xoffset = 0
        line_count = 0
        if len(l) > 0:
            for i in l:
                context.drawText(QtCore.QPoint(100+xoffset, 300+yoffset), str(i.id)+ "  "+str(i.alpha))
                yoffset += 12
                line_count += 1
                if line_count > 5:
                    xoffset += 120
                    yoffset = 0
                    line_count = 0

    def draw_image(self, context):
        context.drawImage(QtCore.QPoint(0,0), self.current_image)

    def draw_all(self, paint_device):
        context = QtGui.QPainter()
        context.begin(paint_device)
        self.draw_image(context)
        if self.data_type == 0:
            self.draw_circles(context)
        elif self.data_type == 1:
            self.draw_lines(context)
            self.draw_info(context)
        context.end()

    def save_framebuffer(self):
        image = self.grabFrameBuffer()
        image.save("img_"+str("%04d" % self.current_step)+".png")

    def paintEvent(self, event):
        self.draw_all(self)

    def grabFrameBuffer(self):
        pixmap = QtGui.QPixmap( self.width(), self.height() )
        self.draw_all(pixmap)
        return pixmap.toImage()

if __name__ == "__main__":
    print "Testing"
    d = Discrete_Step()
    c = Circular_Landmark()
    c.x = 350
    c.y = 200
    d.landmarks.append( c )
    l = Line_Landmark()
    l.alpha = 0.3
    l.beta = 0.3
    l.id = [1,2]
    d.landmarks.append( l )
    d.image = 'largemap240803e.0001.jpg'
    _data = [ d, d ]
    app = QtGui.QApplication(sys.argv)
    image_display = Image_Display()
    image_display.set_image_path('test_images')
    image_display.set_data(_data)
    image_display.show()
    sys.exit(app.exec_())
