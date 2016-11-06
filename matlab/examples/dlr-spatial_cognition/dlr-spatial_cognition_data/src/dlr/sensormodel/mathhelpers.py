#! /usr/bin/env python
# -*- coding: utf-8 -*-

import math

J_over_I = [
[1, 0, 0, 0, 0, 0, 0, 0, 0],
[1, 1, 0, 0, 0, 0, 0, 0, 0],
[1, 2, 1, 0, 0, 0, 0, 0, 0],
[1, 3, 3, 1, 0, 0, 0, 0, 0],
[1, 4, 6, 4, 1, 0, 0, 0, 0],
[1, 5,10,10, 5, 1, 0, 0, 0],
[1, 6,15,20,15, 6, 1, 0, 0],
[1, 7,21,35,35,21, 7, 1, 0],
[1, 8,28,56,70,56,28, 8, 1]]

def distance_to_polynominal_root(a, m, x0):
    # assert (m<= 8)
    ax = [ 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0,
           0.0, 0.0, 0.0 ]
    for i in range(m+1):
        xOPJ = 1.0
        for j in range(i+1):
            ax[i-j] += a[i] * J_over_I[i][j]*xOPJ
            xOPJ *= x0
    max_delta =  float('inf')
    for i in range(1, m+1):
        radicant = math.fabs( ax[0]) / math.fabs(ax[i])
        delta = math.exp(math.log(radicant)/i)/2
        if delta < max_delta:
            max_delta = delta
    return max_delta


if __name__ == '__main__':
    print distance_to_polynominal_root([1,2,3,4,5,6,8,9,10], 8, 1.2)
