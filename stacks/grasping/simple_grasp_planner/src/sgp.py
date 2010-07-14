#!/usr/bin/env python
#Wrapper for the SGP library
#Copyright 2010 Alexis Maldonado <maldonad@cs.tum.edu>
#Released under the LGPL v3 License


from ctypes import *
from ctypes.util import find_library

#Loading the library
#libname = find_library("sgp")

import commands
libdir = commands.getoutput("rospack find simple_grasp_planner")
libname = libdir + "/lib/libsgp.so"

try:
    if (libname):
        libsgp=CDLL(libname)
    else:
        libsgp=CDLL('../lib/libsgp.so')
except:
    libsgp=CDLL('libsgp.so')



#Defining the Structures
class Point3D(Structure):
    _fields_=[ ("x", c_double), ("y", c_double), ("z",c_double) ]

max_tested_poses =  1000
class HandConfig(Structure):
    _fields_=[ ("alpha", c_double), ("beta", c_double), ("delta_max", c_double) ]

class HandConfList(Structure):
    _fields_=[ ("length", c_int), ("min_score_index", c_int), ("scores", c_double * max_tested_poses), ("configs", HandConfig * max_tested_poses)]


class CovariancePoint(Structure):
    _fields_=[ ("sx", c_double ), ("sxy", c_double ), ("sxz", c_double ),
        ("syx", c_double ), ("sy", c_double ), ("syz", c_double ),
        ("szx", c_double ), ("szy", c_double ), ("sz", c_double ) ]

#Factory functions
def new_Point3D_list(length):
    pl=Point3D*length
    return(pl())

def new_CovariancePoint_list(length):
    cl=CovariancePoint*length
    return(cl())

#Function prototypes
libsgp.InitSGP.argtypes = [POINTER(Point3D), c_int, c_double]
libsgp.setTableHeight.argtypes = [c_double]
libsgp.setTableHeight.restype = None
libsgp.getTableHeight.argtypes = []
libsgp.getTableHeight.restype = c_double
libsgp.GetGraspLM.argtypes = [POINTER(Point3D), POINTER(CovariancePoint), c_int, c_double, c_double]
libsgp.GetGraspLM.restype = HandConfig

libsgp.GetGraspList.argtypes = [POINTER(Point3D), POINTER(CovariancePoint), c_int, c_double, c_double]
libsgp.GetGraspList.restype = HandConfList


libsgp.TransformPoint.argtypes = [Point3D, Point3D, c_double, c_double, c_double]
libsgp.TransformPoint.restype = Point3D

if (__name__ == "__main__"):
    print "Doing tests:"
    num_hand_points = 3
    hpl=new_Point3D_list(num_hand_points)
    hpl[1].y = -0.02
    hpl[1].z = 0.05
    hpl[2].y = 0.02
    hpl[2].z = 0.05
    num_objects = 1
    pl=new_Point3D_list(num_objects)
    cl=new_CovariancePoint_list(num_objects)
    cl[0].sx=0.1
    cl[0].sxy=-0.001
    cl[0].sxz=-0.001
    cl[0].sy=0.01
    cl[0].syx=-0.001
    cl[0].syz=-0.001
    cl[0].sz=0.01
    cl[0].szy=-0.001
    cl[0].szx=-0.001
    libsgp.InitSGP(hpl,num_hand_points, -0.1)
    offset_rot_z_side=1.0
    offset_rot_z_top=1.0
    conf = libsgp.GetGraspLM(pl,cl,num_objects, offset_rot_z_side, offset_rot_z_top)
    print conf.alpha
    print conf.beta
    print conf.delta_max
