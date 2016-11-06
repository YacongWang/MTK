#!/usr/bin/env python
 # -*- coding: utf-8 -*-

import psyco
psyco.log()
psyco.full()



from numpy import *
from numpy.linalg import inv, cholesky
from scipy import linsolve, sparse

from dlr.importer import *
from dlr.modelfunctions import *
import cPickle

from slam2D.mapdisplay import Map_Display, EKF_SLAM_Display
from slam2D.helpers import *

import time
import math
import sys
import cPickle

### configure global dtype?!?!
DTYPE=float64


class EKF_Slam:
    def __init__(self):
        self.map = {}  # (id: [ lx, ly, (cov_slice_tuple) ]

        self.mean = matrix( [ [0.0], [0.0], [0.0] ], dtype=float64 ) ### dynamisch wachsend
        #self.cov = matrix( zeros( (3,3), dtype=float64) )
        self.cov = matrix( diag( (0.05*0.05, 0.05*0.05, 0.08726646259971647885*0.08726646259971647885) ) )


        self.pos_idx = (slice(0,3))
        self.pcov_idx = (slice(0,3), slice(0,3))
        self.next_in_cov = (3,3)
        self.step = 0


    def update_map(self):
        for i in self.map.itervalues():
            lslice = i[2][0]
            i[0] = self.mean[ lslice.start, 0 ]
            i[1] = self.mean[ lslice.start +1, 0 ]

    def ekf_slam(self, odometry_t2, odometry_t2_cov, z_t2 ): # all type matrix or list of matrix
        ## alle new_flags löschen
        #for i in self.map.itervalues():
        #    i[3] = False
        self.step+= 1
        print "Step: ", self.step
        J1 = j1( self.mean.T.tolist()[0], odometry_t2.T.tolist()[0] )
        J2 = j2( self.mean.T.tolist()[0] )

        self.mean[ self.pos_idx ] = f1( self.mean.T.tolist()[0], odometry_t2)

        self.cov[self.pcov_idx] = (J1 * self.cov[self.pcov_idx] * J1.T) + (J2 * odometry_t2_cov * J2.T) # J1*E*J1.T + J2*COV(u)*J2.T
        C_lp = ( slice( 3, self.cov.shape[1]), slice(0,3) )
        C_pl = ( slice( 0, 3), slice(3, self.cov.shape[0]))
        self.cov[C_pl] = J1 * self.cov[C_pl]
        self.cov[C_lp] = self.cov[C_lp] * J1.T

        for z in z_t2:
            ## noch nie gesehen?
            if not z[2] in self.map:
                m = f4(self.mean[self.pos_idx].T.tolist()[0], z[0:2])  ## 2D!
                idx = ( slice( self.next_in_cov[0], self.next_in_cov[0]+2),
                        slice( self.next_in_cov[1], self.next_in_cov[1]+2) )
                self.next_in_cov = ( self.next_in_cov[0]+2, self.next_in_cov[1]+2)
                zm = [ m[0,0], m[1,0], idx ]
                self.map[ z[2] ] = zm
                self.mean = vstack( (self.mean, matrix( zm[0:2] ).T) )

                ## from Smith, Self, Cheeseman "Estimating Uncertain Spatial Relationship in Robotics
                J7 = df4_dp( self.mean[self.pos_idx].T.tolist()[0], zm[0:2] )
                J8 = df4_dm( self.mean[self.pos_idx].T.tolist()[0], zm[0:2] )

                J7_0 = hstack( (J7, zeros( (J7.shape[0], self.cov.shape[1] - J7.shape[1])) ) )

                A = J7_0*self.cov*J7_0.T + J8*z[3]*J8.T
                B = J7_0*self.cov
                # row = B
                column = vstack( (B.T, A) )
                cov_tmp = vstack( (self.cov, B) )
                self.cov = hstack( (cov_tmp, column))
            else:
                map_item = self.map[ z[2] ]
                l = map_item[0:2]
                z_bar = f3( self.mean[self.pos_idx].T.tolist()[0], l)
                zt = matrix( [ [ z[0] ],
                               [ z[1] ] ] )

                H = zeros( (2, self.cov.shape[1]), dtype=float64 )
                ## Position für J5+J6 in H berechnen

                J5_pos = ( slice(0,2), self.pos_idx)
                H[ J5_pos ] = j5( self.mean[self.pos_idx].T.tolist()[0], l )
                J6_pos = ( slice(0, 2), map_item[2][1] )
                H[ J6_pos ] = j6( self.mean[self.pos_idx].T.tolist()[0], l )
                Q = z[3]
                #print "Mahalanobis: ", z[2], float((zt - z_bar).T * tmp2 * (zt - z_bar))
                K = self.cov * H.T * inv( H * self.cov * H.T  + Q )
                self.mean += K * ( zt - z_bar )
                self.cov  -= K * ( H * self.cov )

        self.update_map()

def main():
    # setting numpy default output
    # using this options breaks reading from file, somehow precision here and precision in matrices are related
    #set_printoptions(threshold=nan, linewidth=nan, precision=3, suppress=True)
    ekfslam = EKF_Slam()
    image_path = 'images/'
    data_path = '.'
    data_basename = ''
    computation_time = []

    ## parameters (have a commandline for that at some point
    show_gui = True
    save_images = False
    write_computation_time = True
    write_ekfslam_map = True


    if len(sys.argv) == 2:
        data_info = Data_Loader(sys.argv[1], remove_unknown=True)
    else:
        return
    if show_gui:
        md = EKF_SLAM_Display(1024,768, center= (-160,0), scale_x= 0.08, scale_y=0.08)
        md.image_path = data_info.image_path
        md.draw_cov = False
        if save_images:
            time_string = time.strftime('%Y_%m_%d__%H_%M_%S_', time.localtime() )
            md.set_savename( 'ekf_pics/ekf_run_'+time_string )
    for d in data_info.data:
        z = []
        for l in d.landmarks:
            #print l.cov
            #print "Step Cov"
            #print d.step_cov
            z.append ( [l.x, l.y, l.id, l.cov] )

        start_time = time.time()
        ekfslam.ekf_slam(d.step, d.step_cov, z)
        computation_time.append( (time.time() - start_time, len(ekfslam.map) ) )

        if show_gui:
            md.robot = (ekfslam.mean[0,0], ekfslam.mean[1,0], ekfslam.mean[2,0], zeros((3,3)))
            md.map = ekfslam.map
            md.covariance = ekfslam.cov
            md.image = d.image
            md.info['Step'] = ekfslam.step
            md.info['Robot'] = md.robot[0:3]
            md.info['Odometry'] = d.step.T.tolist()[0]
            md.info['Num of z'] = len(d.landmarks)
            md.update()
            #md.wait_for_signal()
            #time.sleep(0.08)
    if write_ekfslam_map:
        counter = 0
        while ( os.path.exists( "ekfslam_"+str(counter)+".map") ):
            counter += 1
        ekfmap = open("ekfslam_"+str(counter)+".map", 'w')
        if ekfmap:
            print "Writing EKF Map"
            ekfmap_flat = {}
            for item in ekfslam.map.iteritems():
                ekfmap_flat[ item[0] ] = [ item[1][0], item[1][1], ekfslam.cov[item[1][2]] ]
            p = cPickle.Pickler(ekfmap)
            p.dump(ekfmap_flat)
            ekfmap.close()
    if write_computation_time:
        counter = 0
        while ( os.path.exists( "ekf_computation_time_"+str(counter)+".map") ):
            counter += 1
        computation_time_file = open("ekf_computation_time_"+str(counter)+".map", 'w')
        if computation_time_file:
            print "Writing computation time array"
            p = cPickle.Pickler(computation_time_file)
            p.dump(computation_time)
            computation_time_file.close()
    if show_gui:
        md.wait_for_quit()

if __name__ == '__main__':
    main()
