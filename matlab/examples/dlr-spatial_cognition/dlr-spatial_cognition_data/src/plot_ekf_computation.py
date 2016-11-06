#!/usr/bin/env python
 # -*- coding: utf-8 -*-

import cPickle
from pylab import *


f = open('ekf_computation_time')
p = cPickle.Unpickler(f)

data = p.load()

num_landmarks = []
computation_time = []
time_sum = 0

for i in data:
    time_sum += i[0]
    computation_time.append( i[0] )
    num_landmarks.append( i[1] )

mean = time_sum / len(data)

print "Computation time (complete):", time_sum
print "Mean:", mean


coeff = polyfit(num_landmarks, computation_time, 3)
best_y = polyval(coeff, num_landmarks)
plot(num_landmarks, computation_time, 'gd', num_landmarks, best_y, '-r', linewidth=2)
grid(True)
ylabel('Time in sec')
xlabel('number of landmarks in map')
show()
