#!/usr/bin/env python
# -*- coding: utf-8 -*-

from numpy import *
from pylab import *
from matplotlib.patches import Ellipse
from matplotlib.text import Annotation


class Easy_Plot:
    ## static for the name of savefile
    save_id = 0
    def __init__(self):
        pass

    def plot_position(self, x_p, format='r+'):
        p_x = []
        p_y = []
        m_x = []
        m_y = []
        for i in x_p:
            p_x.append( i[0] )
            p_y.append( i[1] )
        plot(p_x,p_y, format)

    def plot_landmarks(self, x_l, format='g+'):
        px = []
        py = []
        for i in x_l:
            u,v = i[0], i[1]
            px.append(u)
            py.append(v)
        plot( px, py, format )

    def plot_map_and_pos(self, map, x, save=False):
        ax = subplot(111, axisbg=(1.0,1.0,1.0) , aspect='equal' )
        plot(x[0], x[1], format='rD')
        self.plot_landmarks( map, format='wD')
        draw()
        if ( save ):
            filename = "map"+"%05d" % self.save_id
            self.save_id += 1
            gcf().savefig( filename+".png" )


    def plot_full_map(self, xt_p, xt_l, format_xp = 'r+', format_xl = 'c+', save=False):
        ax = subplot(111, axisbg=(1.0,1.0,1.0) , aspect='equal' )
        self.plot_position(xt_p, format=format_xp)
        self.plot_landmarks( xt_l, format=format_xl)
        draw()
        if ( save ):
            filename = "map"+"%05d" % self.save_id
            self.save_id += 1
            gcf().savefig( filename+".png" )



#####################

class Plot:
    fig = figure(facecolor='white')
    ax = subplot(111, axisbg=(1.0,1.0,1.0) , aspect='equal' )

    def __init__(self):
        ion()
        #show()
        self.ax.set_title('Levenberg-Marquardt Method')

    def plot_landmarks (self, x_l, format='ro', covariance=False):
        """ """
        px = []
        py = []
        for i in x_l:
            u,v = i[0], i[1]
            px.append(u)
            py.append(v)
            l, = self.ax.plot( px, py, format )
            a = self.ax.annotate(str(i[2]), xy=[i[0], i[1]], xycoords='data') #, textcoords='offset points')
            self.ax.add_artist(a)
        if covariance:
            for i in x_l:
                v, m = eig(i[3])
                ## Länge und Winkel herauskriegen
                l1 = sqrt(v[0])*2
                l2 = sqrt(v[1])*2
                phi1 = atan2( m[1,0], m[0,0] )
                #phi2 = arccos( m[0,1] / sqrt( m[0,1]*m[0,1] + m[1,1]*m[1,1] ) )
                angle = phi1 * 180 / pi
                e = Ellipse( [i[0], i[1]], l1, l2, angle, fill=False)
                self.ax.add_artist(e)
                xdata = [ i[0] - l1/ 2*cos(phi1), i[0] +l1 /2*cos(phi1) ]
                ydata = [ i[1] - l1/ 2*sin(phi1), i[1] +l1 /2*sin(phi1) ]
                l.set_xdata = xdata
                l.set_ydata = y_data
                self.ax.add_artist(l)

    def show(self):
        self.fig.show()

    def draw(self):
        draw()
        #show()

    def clear(self):
        self.fig.clear()

    def save_plot(self):
        self.fig.savefig('map.png')

    def plot_position(self, x_p, format='r+'):
        p_x = []
        p_y = []
        m_x = []
        m_y = []
        for i in x_p:
            p_x.append( i[0] )
            p_y.append( i[1] )
        plot(p_x,p_y, format)

    def update_map(self, x_p, xl):
        p_x = []
        p_y = []
        for i in x_p:
            p_x.append( i[0] )
            p_y.append( i[1] )
        self.l.set_xdata = p_x
        self.l.set_ydata = p_y
        draw()

    def plot_map(self, x0_p, x0_l, xt_p, xt_l, format_xp = 'r+', format_xl = 'g+'):
        #self.fig.clear()
        #self.plot_landmarks( x0_l)
        self.plot_position(x0_p)
        self.plot_position(xt_p)
        #self.plot_landmarks( xt_l, format=format_xl)
        #self.l = self.plot_position(xt_p, format=format_xp)
        draw()


