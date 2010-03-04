#!/usr/bin/python

import math

def poisson_pdf(x, lam):
    return lam*math.exp(-lam*x)

def poisson_cdf(x, lam):
    return 1 - math.exp(-lam*x)

def poisson_quantile(p, lam):
    return -math.log(1 - p)/lam

if __name__ == '__main__':
    pass

