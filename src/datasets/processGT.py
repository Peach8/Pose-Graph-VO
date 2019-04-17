# -*- coding: utf-8 -*-
"""
Created on Sun Apr 14 13:56:41 2019

@author: winni
"""
import numpy as np
import scipy.io as sio

data = []
xyz = []
GT_file = open('freiburg3_groundtruth_md.txt','r')
#line = GT_file.readlines()
cnt = 0
while cnt < 8710:
   line = GT_file.readline()
   [time, tx, ty, tz, qx, qy, qz, qw] = line.split(' ')
   data.append([float(time), float(tx), float(ty), float(tz), float(qx),float(qy),float(qz),float(qw)])
#   xyz.append([float(tx), float(ty), float(tz)])
   cnt += 1

data = np.array(data)

sio.savemat('freiburg3_gtcoord.mat', mdict={'freiburg3_gtcoord': data})