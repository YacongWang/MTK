import importer

from pylab import *
from numpy import *
import random


c = []
g_max = 0.0
g_min = 0.0

for i in range(4000):
    g = random.gauss(0,2)
    c.append( g )
    if g < g_min:
        g_min = g
    if g > g_max:
        g_max = g

a = arange(g_min, g_max, 0.1)
b = []
for i in range(0, len(a)):
    b.append(0)

for i in c:
    for j,k in enumerate(a):
        if ( i < k and i > a[j-1] ):
            b[j] += 1
            continue

print b

plot(a, b)
show()
