# -*- coding: utf-8 -*-
"""
Created on Sun Apr 14 13:56:41 2019

@author: winni
"""
import numpy as np
import scipy.io as sio

data = []
xyz = []
GT_file = open('freiburg2_groundtruth.txt','r')
cnt = 0
while cnt < 32992:
   line = GT_file.readline()
   [time, tx, ty, tz, qx, qy, qz, qw] = line.split(' ')
   data.append(line)
   xyz.append([float(tx), float(ty), float(tz)])
   cnt += 1

xyz = np.array(xyz)

sio.savemat('11.mat', mdict={'xyz': xyz})