#! /usr/bin/env python

import roslib
roslib.load_manifest('image_algos')
import rospy

print "OpenCV Python version of lkdemo"

import sys, math

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

class FeatureDists:
    feature = (0.0, 0.0)
    dists = []
        
#############################################################################
# so, here is the main part of the program
        
if __name__ == '__main__':
    
    frames = sys.argv[1:]
    if frames == []:
        print "usage lkdemo.py <image files>"
        sys.exit(1)
        
    # display a small howto use it
    print "Hot keys: \n" \
        "\tESC - quit the program\n" \
        "\tr - auto-initialize tracking\n" \
        "\tc - delete all the points\n" \
        "\tn - switch the \"night\" mode on/off\n" \
        "\tSPACE - next frame\n" \
        "To add/remove a feature point click it\n"
    
    # first, create the necessary windows
    cv.NamedWindow ('LkDemo', cv.CV_WINDOW_AUTOSIZE)
    # cv.NamedWindow ('grey', cv.CV_WINDOW_AUTOSIZE)
    # cv.NamedWindow ('prev_grey', cv.CV_WINDOW_AUTOSIZE)

    # register the mouse callback
    cv.SetMouseCallback ('LkDemo', on_mouse, None)
    
    fc = 0
    while 1:
        # do forever
        
        frame = cv.LoadImage(frames[fc])
        
        if image is None:
            # create the images we need
            image = cv.CreateImage (cv.GetSize (frame), 8, 3)
            image.origin = frame.origin
            grey = cv.CreateImage (cv.GetSize (frame), 8, 1)
            prev_grey = cv.CreateImage (cv.GetSize (frame), 8, 1)
            pyramid = cv.CreateImage (cv.GetSize (frame), 8, 1)
            prev_pyramid = cv.CreateImage (cv.GetSize (frame), 8, 1)
            features = []
            
        # copy the frame, so we can draw on it
        cv.Copy (frame, image)
        
        # create a grey version of the image
        cv.CvtColor (image, grey, cv.CV_BGR2GRAY)
        
        if night_mode:
            # night mode: only display the points
            cv.SetZero (image)
            
        if need_to_init:
            # we want to search all the good points
            
            # create the wanted images
            eig = cv.CreateImage (cv.GetSize (grey), 32, 1)
            temp = cv.CreateImage (cv.GetSize (grey), 32, 1)
            
            # the default parameters
            quality = 0.01
            min_distance = 10
            
            # search the good points
            features = cv.GoodFeaturesToTrack (
                grey, eig, temp,
                MAX_COUNT,
                quality, min_distance, None, 3, 0, 0.04)
            
            # refine the corner locations
            features = cv.FindCornerSubPix (
                grey,
                features,
                (win_size, win_size),  (-1, -1),
                (cv.CV_TERMCRIT_ITER | cv.CV_TERMCRIT_EPS, 20, 0.03))
            
        elif features != []:
            # we have points, so display them
            # cv.ShowImage ('prev_grey', prev_grey)
            # cv.ShowImage ('grey', grey)

            # calculate the optical flow
            features, status, track_error = cv.CalcOpticalFlowPyrLK (
                prev_grey, grey, prev_pyramid, pyramid,
                features,
                (win_size, win_size), 3,
                (cv.CV_TERMCRIT_ITER|cv.CV_TERMCRIT_EPS, 20, 0.03),
                flags)
#            print "num features: ", sum(status)
#            print "status: ", status

            # set back the points we keep
            features = [ p for (st,p) in zip(status, features) if st]
            
            # count = 0;
            # for feat in features:
            #     print "feature coordinates: ", feat, count
            #     count = count + 1

            if add_remove_pt:
                # we have a point to add, so see if it is close to
                # another one. If yes, don't use it
                def ptptdist(p0, p1):
                    dx = p0[0] - p1[0]
                    dy = p0[1] - p1[1]
                    return dx**2 + dy**2
                if min([ ptptdist(pt, p) for p in features ]) < 25:
                    # too close
                    add_remove_pt = 0
                    
            # draw the points as green circles
            for the_point in features:
                print  "feature coordinates: ", the_point[0], the_point[1] 
                cv.Circle (image, (int(the_point[0]), int(the_point[1])), 3, (0, 255, 0, 0), -1, 8, 0)


            count = 0
            dist_list = []
            for the_point in features:
                for p in features:
                    if the_point != p:
                        dist = math.sqrt(math.pow(the_point[0] - p[0], 2) + math.pow(the_point[1] - p[1], 2))
                        dist_list.append(dist)
                print "new list: ", dist_list
                dist_list = []
            # calculate distance to other features
            #for the_point in features:
                

        if add_remove_pt:
            # we want to add a point
            # refine this corner location and append it to 'features'
            
            features += cv.FindCornerSubPix (
                grey,
                [pt],
                (win_size, win_size),  (-1, -1),
                (cv.CV_TERMCRIT_ITER | cv.CV_TERMCRIT_EPS,
                 20, 0.03))
            # we are no longer in "add_remove_pt" mode
            add_remove_pt = False
            
        # swapping
        prev_grey, grey = grey, prev_grey
        prev_pyramid, pyramid = pyramid, prev_pyramid
        need_to_init = False
        
        # we can now display the image
        cv.ShowImage ('LkDemo', image)
        
        # handle events
        c = cv.WaitKey (0)
        
        if c == 27:
            # user has press the ESC key, so exit
            break
        
        #print fc, len(frames)
        if fc == len(frames)-1:
            break

        # processing depending on the character
        if 32 <= c and c < 128:
            cc = chr(c).lower()
            if cc == 'r':
                need_to_init = True
            elif cc == 'c':
                features = []
            elif cc == 'n':
                night_mode = not night_mode
            elif cc == ' ':
                fc = (fc + 1) % len(frames)
                
