#! /usr/bin/env python

import roslib
roslib.load_manifest('image_algos')
import rospy

print "OpenCV Python version of lkdemo"

import sys
import urllib

# import the necessary things for OpenCV
import cv

#############################################################################
# some "constants"

win_size = 10
MAX_COUNT = 500

#############################################################################
# some "global" variables

image = None
pt = None
add_remove_pt = False
flags = 0
night_mode = False
need_to_init = False

#############################################################################
# the mouse callback

# the callback on the trackbar
def on_mouse (event, x, y, flags, param):
    
    # we will use the global pt and add_remove_pt
    global pt
    global add_remove_pt
    
    if image is None:
        # not initialized, so skip
        return
    
    if image.origin != 0:
        # different origin
        y = image.height - y
        
    if event == cv.CV_EVENT_LBUTTONDOWN:
        # user has click, so memorize it
        pt = (x, y)
        add_remove_pt = True

def snapL(L):
    for i,img in enumerate(L):
        cv.NamedWindow("snap-%d" % i, 1)
        cv.ShowImage("snap-%d" % i, img)
        cv.WaitKey()
        cv.DestroyAllWindows()        

def get_sample(filename, iscolor = cv.CV_LOAD_IMAGE_COLOR):
    image_cache = {}
    if not filename in image_cache:
        #filedata = filename
        imagefiledata = cv.CreateMatHeader(1, len(filedata), cv.CV_8UC1)
        cv.SetData(imagefiledata, filedata, len(filedata))
        image_cache[filename] = cv.DecodeImageM(imagefiledata, iscolor)
        return image_cache[filename]

#############################################################################
# so, here is the main part of the program
        
if __name__ == '__main__':
    
    # frames = sys.argv[1:]
    # if frames == []:
    #     print "usage lkdemo.py <image file1> <image file2>"
    #     sys.exit(1)
    frames = sys.argv[1:]
    if frames == []:
        print "usage track_features_lk2.py <image files>"
        sys.exit(1)

    fc = 0
    while 1:
        # do forever
        
        aa = cv.LoadImage(frames[fc])
        bb = cv.LoadImage(frames[fc+1])
        a = cv.CreateImage (cv.GetSize (aa), 8, 1)
        b = cv.CreateImage (cv.GetSize (aa), 8, 1)
        cv.CvtColor (aa, a, cv.CV_BGR2GRAY)
        cv.CvtColor (bb, b, cv.CV_BGR2GRAY)
        # map = cv.CreateMat(2, 3, cv.CV_32FC1)
        # cv.GetRotationMatrix2D((256, 256), 10, 1.0, map)
        # b = cv.CloneMat(a)
        # cv.WarpAffine(a, b, map)
        
        eig_image = cv.CreateMat(a.height, a.width, cv.CV_32FC1)
        temp_image = cv.CreateMat(a.height, a.width, cv.CV_32FC1)
        
        prevPyr = cv.CreateMat(a.height / 3, a.width + 8, cv.CV_8UC1)
        currPyr = cv.CreateMat(a.height / 3, a.width + 8, cv.CV_8UC1)
        prevFeatures = cv.GoodFeaturesToTrack(a, eig_image, temp_image, 400, 0.01, 0.01)
        (currFeatures, status, track_error) = cv.CalcOpticalFlowPyrLK(a,
                                                                      b,
                                                                      prevPyr,
                                                                      currPyr,
                                                                      prevFeatures,
                                                                      (10, 10),
                                                                      3,
                                                                      (cv.CV_TERMCRIT_ITER|cv.CV_TERMCRIT_EPS,20, 0.03),
                                                                      0)
        if 1:  # enable visualization
            print
            print sum(status), "Points found in curr image"
            for prev,this in zip(prevFeatures, currFeatures):
                iprev = tuple([int(c) for c in prev])
                ithis = tuple([int(c) for c in this])
                cv.Circle(a, iprev, 3, 255)
                cv.Circle(a, ithis, 3, 0)
                cv.Line(a, iprev, ithis, 128)
                
            snapL([a, b])

        fc = (fc + 1) % len(frames)
        #exit
        if fc == len(frames):
            break
