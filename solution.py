#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Sep  4 11:43:26 2023

@author: stonneau
"""

from os.path import dirname, join, abspath
import numpy as np
import pinocchio as pin #the pinocchio library
from pinocchio.utils import rotate

#helpers 
#if needed, you can store the placement of the right hand in the left hand frame here

R = np.array([[-0.999999, 0.00159194, -3.01527e-07],
              [-0.00159194, -0.999999, -7.40736e-07],
              [-3.02706e-07, -7.40255e-07, 1]])
LMRREF = pin.SE3(pin.Quaternion(R), np.array([8.32116e-07, -0.100001, 8.62878e-08]))
RMLREF = LMRREF.inverse()
L_nu = pin.log(LMRREF).vector
R_nu = pin.log(RMLREF).vector

GRASP_DISTANCE_THRESHOLD = 5e-2

# R = np.array([[-9.99459247e-01, -3.28818233e-02, -4.54084458e-17]
#              ,[ 3.28818233e-02, -9.99459247e-01, -6.16983920e-17]
#              ,[-5.29195017e-17, -5.50829838e-17,  1.00000000e+00]])
# LMRREF = pin.SE3(pin.Quaternion(R), np.array(np.array([-6.92210039e-03, -1.01809409e-01,  4.72468083e-06])))