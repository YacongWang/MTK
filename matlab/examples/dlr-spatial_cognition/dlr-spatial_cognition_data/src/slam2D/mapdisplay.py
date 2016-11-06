import pygame
from pygame.locals import *
import sys
from numpy import *
from pylab import *
from math import sin,cos,pi,atan2
import os
import os.path

def draw_ellipse( surface, (x,y), (axis1, axis2), alpha, color = (255,255,255), step = 2*pi/8, scaler = None ):
    pointlist = []
    for theta in arange(0, 2 * pi, step):
        r = matrix( [ [cos(alpha), -sin(alpha)],
                      [sin(alpha),  cos(alpha)] ] )
        d = matrix( [ [axis1*cos(theta)],[axis2*sin(theta)] ] )
        point = matrix([ [x],[y] ]) + r * d
        pointlist.append( ( point[0,0], point[1,0] ) )
    pygame.draw.polygon(surface, color, pointlist, 1)


class Map_Display:
    def __init__(self, width, height, scale_x = 0.1, scale_y = 0.1, center= (0,0), bg_color=(0,0,0), map_color=(255, 0,0) ):
        pygame.init()
        if not pygame.font.get_init():
            pygame.font.init()
        self.font = pygame.font.Font("/usr/share/fonts/truetype/ttf-bitstream-vera/VeraMono.ttf", 12)

        self.size = self.width, self.height = width, height
        self.scale = self.scale_x, self.scale_y =  self.width*scale_x, self.height*scale_y
        self.center = (self.width/2 + center[0], self.height/2+ center[1])
        self.bg_color = bg_color
        self.map_color = map_color
        self.screen = pygame.display.set_mode(self.size)
        self.grid = False
        self.scale = False
        self.map = []
        self.robot = (0.0, 0.0, 0.0, zeros((3,3), dtype=float64))
        self.prediction = matrix( [ [0.0], [0.0], [0.0] ] )
        self.info = dict( [] )
        self.real_pos = []
        self.step = 0
        self.savename = ''

    def set_savename(self, savename):
        self.savename = savename

    def save(self):
        f = "%s_%04d.tga" % (self.savename, self.step)
        #print "Saving screenshot to file", f
        pygame.image.save(self.screen, f )

    def to_screen(self, p):
        x = ((self.width * -p[0]) / self.scale_x) + self.center[0]
        y = ((self.height * p[1]) / self.scale_y) + self.center[1]
        return int(x),int(y)

    def update(self):
        self.screen.fill(self.bg_color)
        # Gitter zeichnen?
        self.draw_map()
        self.draw_robot()
        self.draw_info()
        pygame.display.flip()
        if self.savename != '':
            self.save()
        self.step += 1

    def draw_prediction(self):
        pygame.draw.circle(self.screen, (0,255,255),
                           self.to_screen( (self.prediction[0,0], self.prediction[1,0]) ),
                           3)

    def draw_real_pos(self):
        pygame.draw.circle(self.screen, (255,255,255),
                           self.to_screen( (self.real_pos[self.step][0], self.real_pos[self.step][1]) ),
                           3)




    def draw_robot(self):
        pygame.draw.circle(self.screen, (0,255,0),
                           self.to_screen( (self.robot[0], self.robot[1]) ),
                           3)


    def draw_map(self):
        for point in self.map:
            pygame.draw.circle(self.screen, self.map_color,
                               self.to_screen( (point[0], point[1]) ),
                               1)

    def draw_info(self):
        pos = [1,1]
        for t,i in self.info.iteritems():
            font_surface = self.font.render( t+ ": " + str(i), False, (255,255,255))
            self.screen.blit(font_surface, pos)
            pos[1] += font_surface.get_height()

    def wait_for_quit(self):
        while 1:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    sys.exit()

    def wait_for_signal(self):
        while 1:
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:
                    return
                if event.type == pygame.QUIT:
                    sys.exit()

class EKF_SLAM_Display(Map_Display):
    def __init__(self, width, height, scale_x = 0.1, scale_y = 0.1, center= (0,0), bg_color=(0,0,0), map_color=(255, 0,0)):
        Map_Display.__init__(self, width, height, scale_x, scale_y, center, bg_color, map_color)
        self.map = {}
        self.covariance = matrix( zeros((3,3)) )
        self.image_path = None
        self.draw_cov = True

    def draw_covariance(self, point, cov):
        v, m = eig(cov)
        ## Laenge und Winkel herauskriegen
        l1 = sqrt(v[0])*2
        l2 = sqrt(v[1])*2
        #print "Axis:", (l1, l2)
        phi1 = atan2( m[1,0], m[0,0] )
        angle = phi1 * 180 / pi
        scale = 60.0
        draw_ellipse(self.screen, self.to_screen( (point[0], point[1])), (scale*l1, scale*l2), phi1, (255,255,255), 0.1)

    def draw_image(self):
        if self.image_path:
            self.image_surface = pygame.image.load( os.path.join( self.image_path, self.image+".jpg" ) )
            self.image_surface = pygame.transform.scale(self.image_surface, (self.image_surface.get_width()/2, self.image_surface.get_height()/2))
            self.screen.blit(self.image_surface, (self.screen.get_width()-self.image_surface.get_width(), 0))

    ## overwrite draw map
    def draw_map(self):
        self.draw_image()
        for point in self.map.itervalues():
            color = self.map_color
            #if point[3]:
            #    color = (255, 255,0) # gelb?!
            pygame.draw.circle(self.screen, color,
                               self.to_screen( (point[0], point[1]) ),
                               1)
            if self.draw_cov:
                if isinstance(point[2][0], slice):
                    cov = self.covariance[point[2]]
                else:
                    cov = point[2]
                self.draw_covariance( (point[0], point[1]),  cov )

