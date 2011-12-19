#! /usr/bin/env python

import roslib
import geometry_msgs


roslib.load_manifest('pr2_interactive_segmentation')
import rospy



import sys, math

# import the necessary things for OpenCV
import cv
import copy
import numpy
import pr2_interactive_segmentation.srv
#############################################################################
# some "constants"

win_size = 5
MAX_COUNT = 500

#############################################################################
# some "global" variables

image = None
pt = None
flags = 0
night_mode = False
need_to_init = False
pt_selected = False


col = (numpy.random.randint(256, size = 500 * 3)).reshape(500,3)


#############################################################################
# the mouse callback

# the callback on the trackbar
def on_mouse (event, x, y, flags, param):
    
    # we will use the global pt
    global pt
    global pt_select
    global pt_selected

    if image is None:
        # not initialized, so skip
        return
    
    if image.origin != 0:
        # different origin
        y = image.height - y
        
    if event == cv.CV_EVENT_LBUTTONDOWN:
        # user has click, so memorize it
        pt = (x, y)
        pt_select = (x, y)
        print "pt_select: ", pt_select
        pt_selected = True

class FeatureDists:
    feature = (0.0, 0.0)
    dists = []
        
#############################################################################
# so, here is the main part of the program
        
class FeatureTracker:
    
    def __init__(self):
        
                # display a small howto use it
        print "Hot keys: \n" \
            "\tq - quit the program\n" \
            "\tr - auto-initialize tracking\n" \
            "\tc - delete all the points\n" \
            "\tn - switch the \"night\" mode on/off\n" \
            "\tSPACE - next frame\n" \
        
        # first, create the necessary windows
        cv.NamedWindow ('LkDemo', cv.CV_WINDOW_NORMAL)
    
        # register the mouse callback
        cv.SetMouseCallback ('LkDemo', on_mouse, None)
        self.win_size = 15
        self.rigid_tracking = False
        self.estimateRigidServer = rospy.ServiceProxy("estimateRigid", pr2_interactive_segmentation.srv.estimateRigid())
        self.getDepthServer = rospy.ServiceProxy("getDepthImage", pr2_interactive_segmentation.srv.depthImage())
        self.ROI = [0,412,2190,1424] #roi for prosilica x,y,width,height
		
        
    
    def initFirstFrame(self, frame, mask = None):
        print "initFirstFrame"
        self.image = cv.CreateImage (cv.GetSize (frame), 8, 3)
#        self.image.origin = frame.origin
        self.grey = cv.CreateImage (cv.GetSize (frame), 8, 1)
        self.prev_grey = cv.CreateImage (cv.GetSize (frame), 8, 1)
        self.pyramid = cv.CreateImage (cv.GetSize (frame), 8, 1)
        self.prev_pyramid = cv.CreateImage (cv.GetSize (frame), 8, 1)
        self.features = []
        #self.feature_object_index = [0] * len(all_features)
	roimask = None
	if self.ROI != None:
	    roimask = cv.CreateImage (cv.GetSize (frame), 8, 1)
	    cv.Zero(roimask)
            cv.Rectangle(roimask, (self.ROI[0],self.ROI[1]), (self.ROI[0] + self.ROI[3],self.ROI[1] + self.ROI[3]), 
                (255,255,255,0),-1)
			
        
        if self.rigid_tracking:
            self.getDepthServer()
        
        image = self.image
        grey = self.grey
        
        cv.Copy (frame, image)
        
        # create a grey version of the image
        cv.CvtColor (image, grey, cv.CV_BGR2GRAY)
        
                # we want to search all the good points
        
        # create the wanted images
        eig = cv.CreateImage (cv.GetSize (grey), 32, 1)
        temp = cv.CreateImage (cv.GetSize (grey), 32, 1)
        
        # the default parameters
        quality = 0.01
        min_distance = 10
        print "finding features"
        # search the good points
        features = cv.GoodFeaturesToTrack (
            grey, eig, temp,
            MAX_COUNT,
            quality, min_distance, roimask, 3, 0, 0.04)
        
        #filter features through mask
        if not mask==None:
	    print "features", features
            features  = [f for f in features if not mask[round(f[1]),round(f[0])] ]
    #        filter(lambda f: not mask[round(f[1])][round(f[0])], features)
            
        
        
        print "refining corners"
        # refine the corner locations
        features = cv.FindCornerSubPix (
            grey,
            features,
            (win_size, win_size),  (-1, -1),
            (cv.CV_TERMCRIT_ITER | cv.CV_TERMCRIT_EPS, 20, 0.03))
        
        
        print "self.features", self.features

	co = cv.RGB(*col[0])
        for the_point in features:
            cv.Circle (image, (int(the_point[0]), int(the_point[1])), 7, co, -1, 8, 0)
            
        
        cv.Copy(grey, self.prev_grey)
        self.image = image
        
        feature_object_index = [0] * len(features)
        
        self.features_with_obj_idx = zip(features, feature_object_index)
        print self.features_with_obj_idx 
        
        return features
        
    
    
    def track(self, frame, mask = None):
        print "track"
#        min_distance_to_pt_select = sys.float_info.max
#        point_id = sys.maxint
#    
#        init_dist_list = []
#        dist_list = []
#        compute_list_diff = False

        prev_grey = self.prev_grey
        pyramid = self.pyramid
        prev_pyramid = self.prev_pyramid


        image = self.image
        grey = self.grey
        
        cv.Copy (frame, image)
        
        # create a grey version of the image
        cv.CvtColor (image, grey, cv.CV_BGR2GRAY)
        
        if night_mode:
            # night mode: only display the points
            cv.SetZero (image)
            
 
        features_old, feature_idx_old = zip(*self.features_with_obj_idx)
        print "features_old",features_old
#        features = self.features
        # calculate the optical flow
    
        print "calculating optical flow"
        features, status, track_error = cv.CalcOpticalFlowPyrLK (
            prev_grey, grey, prev_pyramid, pyramid,
            features_old,
            (self.win_size, self.win_size), 3,
            (cv.CV_TERMCRIT_ITER|cv.CV_TERMCRIT_EPS, 20, 0.03),
            flags)
        print "num features: ", sum(status)
        print "status: ", status
        print "len(status)", len(status)


                #filter features through mask
        if not mask==None:
            features  = [f for f in features if not mask[round(f[1]),round(f[0])] ]


#        self.features = features
        
        for (f_old, f_new) in zip(features_old, features):
            cv.Line(image, f_old, f_new, (0,0,255,0),5)

        #draw features
#        for (st,p) in zip(status, features):
#            if st == 0:
#                 print" st: ", st, "p: ", p
#                 cv.Circle (image, (int(p[0]), int(p[1])), 7, (255, 0, 255, 0), -1, 8, 0)
        
#        count = 0;
#        for feat in features:
#             print "feature coordinates: ", feat, count, type(feat)
#             count = count + 1
             
        print type(features)
#        all_features = cv.fromarray(numpy.asarray(features, dtype='float32')) #copy.deepcopy(features)
#        all_features_old = cv.fromarray(numpy.asarray(features_old, dtype='float32')) #copy.deepcopy(features_old)
#        H = cv.CreateMat(3,3,cv.CV_32F)
        
        
        
        
        
        feature_sets_new = []
        feature_object_index_new = [0] * len(features)
        
        object_count = max(feature_idx_old) + 1
        
        for obj_idx in range(object_count):
            feature_group_new = [f for (f, idx) in zip(features, feature_idx_old) if idx == obj_idx]
            
            if len(feature_group_new) == 0:
                print "empty index !!!!!!!!!!!!! ", obj_idx
                continue
            
            feature_group_old = [f for (f, idx) in self.features_with_obj_idx if idx == obj_idx]
            
            
            
            idx_group_new = [obj_idx] * len(feature_group_new)
            
            print "feature_group_new", feature_group_new
            
            all_features = cv.fromarray(numpy.asarray(feature_group_new, dtype='float32')) #copy.deepcopy(features)
            all_features_old = cv.fromarray(numpy.asarray(feature_group_old, dtype='float32')) #copy.deepcopy(features_old)
            H = cv.CreateMat(3,3,cv.CV_32F)
            
            new_object_count = 0
            #this code below is horrible
            while all_features.height > 4:
                
                status = cv.CreateMat(1,all_features.height,cv.CV_8U)
                if self.rigid_tracking == True:
		    print "using Rigid Motion Estimation"
                    status, success = self.estimateRigidTransformationRANSAC(all_features_old, all_features)
		    status = [status]
		    statusH = cv.CreateMat(1,all_features.height,cv.CV_8U)
		    cv.FindHomography(all_features,all_features_old,H,cv.CV_RANSAC, 5.0 , status = statusH)
                else:
		    print "using Homography"
                    cv.FindHomography(all_features,all_features_old,H,cv.CV_RANSAC, 5.0 , status = status)
                new_object = []
                j = 0
                print "status", numpy.asarray(status)
		print "statusH", numpy.asarray(statusH)
                
                all_features_np = numpy.asarray(all_features)
                all_features_old_np = numpy.asarray(all_features_old)
                
                status_np = numpy.asarray(status)
                print "len(status_np[0])",len(status_np[0])
                
                for i in range(len(status_np[0])):
                    if status_np[0,i]:
#                        print 'adding'
                        new_object.append(tuple(all_features_np[j,:]))
                        all_features_np = numpy.delete( all_features_np, j ,0)
                        all_features_old_np = numpy.delete( all_features_old_np, j ,0)
                    else:
                        j += 1
                if len(new_object) == 0:
                    print "should never get here"
                    break
                
#                print "new_object",new_object
                
                if new_object_count == 0: #we are still in the predominat osiiotn
                    new_idx = obj_idx
                else:
                    new_idx = object_count + new_object_count
                
                feature_sets_new.extend(zip(new_object, [new_idx] * len(new_object)) )
                    
                print "found new object numero new_idx",new_idx
#                print "feature_sets_new",feature_sets_new
                    
                new_object_count += 1
                
                
                all_features = cv.fromarray(all_features_np)
                all_features_old = cv.fromarray(all_features_old_np)
                
                
                co = cv.RGB(*col[new_idx])
                print "co",co
                for the_point in new_object:
                    cv.Circle (image, (int(the_point[0]), int(the_point[1])), 7,co, -1, 8, 0)
		    
#	    if len(all_features_np) > 0:
#		print "unassigned features left"
#		new_object_count += 1
#		new_object = [ (f1,f2) for (
#		feature_sets_new.extend(zip(new_object, [new_object_count] * len(new_object)) )
#		co = cv.RGB(*col[new_object_count])
#		for the_point in new_object:
#		    cv.Circle (image, (int(the_point[0]), int(the_point[1])), 10,co, -1, 8, 0)
    
                    
                 
                
   
            
            
            
            
            
            
        
        
        
#        while all_features.height >= 4:
#            status = cv.CreateMat(1,all_features.height,cv.CV_8U)
#            cv.FindHomography(all_features,all_features_old,H,cv.CV_RANSAC, 3.0 , status = status)
#            new_object = []
#            j = 0
#            
#            all_features_np = numpy.asarray(all_features)
#            all_features_old_np = numpy.asarray(all_features_old)
#            
#            for i in range(status.width):
#                if status[0,i]:
#                    new_object.append(all_features_np[j,:])
#                    all_features_np = numpy.delete( all_features_np, j ,0)
#                    all_features_old_np = numpy.delete( all_features_old_np, j ,0)
#                else:
#                    j += 1
#            if len(new_object) == 0:
#                break
#            feature_sets_new.append(new_object)
#            
#            all_features = cv.fromarray(all_features_np)
#            all_features_old = cv.fromarray(all_features_old_np)
#            
#            col = (numpy.random.randint(256),numpy.random.randint(256),numpy.random.randint(256),0)
#            
#            for the_point in new_object:
#                cv.Circle (image, (int(the_point[0]), int(the_point[1])), 3, col, -1, 8, 0)
                
#        cv.ShowImage('LkDemo', image)
#        cv.WaitKey()
                
    
            
            # swapping
        print "Swapping"
        self.features_with_obj_idx = feature_sets_new
        self.prev_grey, self.grey = grey, prev_grey
        self.prev_pyramid, self.pyramid = pyramid, prev_pyramid
        need_to_init = False
        self.image = image
        return (features, feature_sets_new)
        
        # we can now display the image
#def display(feature_tracker):


    def estimateRigidTransformationRANSAC(self, all_features_old, all_features  ):

     
        req = pr2_interactive_segmentation.srv.estimateRigidRequest()
#        old_points = [geometry_msgs.msg.Point(f[0], f[1], 0.0) for f in all_features_old]
#        new_points = [geometry_msgs.msg.Point(f[0], f[1], 0.0) for f in all_features]
	new_points = []
	old_points = []
	for i in range(all_features.height):
	    new_points.append(geometry_msgs.msg.Point(all_features[i,0], all_features[i,1], 0.0))
	    
	for j in range(all_features_old.height):
	    old_points.append(geometry_msgs.msg.Point(all_features_old[j,0], all_features_old[j,1], 0.0))
	    
        req.features_old = old_points
        req.features_new = new_points
        
        
        res = self.estimateRigidServer(req)
        return res.inliers, res.success
        
        
        
    
    def trackRigid(self, frame, mask = None):
        print "track"
#        min_distance_to_pt_select = sys.float_info.max
#        point_id = sys.maxint
#    
#        init_dist_list = []
#        dist_list = []
#        compute_list_diff = False

        prev_grey = self.prev_grey
        pyramid = self.pyramid
        prev_pyramid = self.prev_pyramid


        image = self.image
        grey = self.grey
        
        cv.Copy (frame, image)
        
        # create a grey version of the image
        cv.CvtColor (image, grey, cv.CV_BGR2GRAY)
        
        if night_mode:
            # night mode: only display the points
            cv.SetZero (image)
            
 
        features_old, feature_idx_old = zip(*self.features_with_obj_idx)
        print "features_old",features_old
#        features = self.features
        # calculate the optical flow
    
        print "calculating optical flow"
        features, status, track_error = cv.CalcOpticalFlowPyrLK (
            prev_grey, grey, prev_pyramid, pyramid,
            features_old,
            (self.win_size, self.win_size), 3,
            (cv.CV_TERMCRIT_ITER|cv.CV_TERMCRIT_EPS, 20, 0.03),
            flags)
        print "num features: ", sum(status), "frame: ", fc
        print "status: ", status
        print "len(status)", len(status)


                #filter features through mask
        if not mask==None:
            features  = [f for f in features if not mask[round(f[1]),round(f[0])] ]


#        self.features = features
        
        for (f_old, f_new) in zip(features_old, features):
            cv.Line(image, f_old, f_new, (0,0,255,0),1)

        #draw features
#        for (st,p) in zip(status, features):
#            if st == 0:
#                 print" st: ", st, "p: ", p
#                 cv.Circle (image, (int(p[0]), int(p[1])), 7, (255, 0, 255, 0), -1, 8, 0)
        
#        count = 0;
#        for feat in features:
#             print "feature coordinates: ", feat, count, type(feat)
#             count = count + 1
             
        print type(features)
#        all_features = cv.fromarray(numpy.asarray(features, dtype='float32')) #copy.deepcopy(features)
#        all_features_old = cv.fromarray(numpy.asarray(features_old, dtype='float32')) #copy.deepcopy(features_old)
#        H = cv.CreateMat(3,3,cv.CV_32F)
        
        
        
        
        
        feature_sets_new = []
        feature_object_index_new = [0] * len(features)
        
        object_count = max(feature_idx_old) + 1
        
        for obj_idx in range(object_count):
            feature_group_new = [f for (f, idx) in zip(features, feature_idx_old) if idx == obj_idx]
            
            if len(feature_group_new) == 0:
                print "empty index !!!!!!!!!!!!! ", obj_idx
                continue
            
            feature_group_old = [f for (f, idx) in self.features_with_obj_idx if idx == obj_idx]
            
            
            
            idx_group_new = [obj_idx] * len(feature_group_new)
            
            print "feature_group_new", feature_group_new
            
            all_features = cv.fromarray(numpy.asarray(feature_group_new, dtype='float32')) #copy.deepcopy(features)
            all_features_old = cv.fromarray(numpy.asarray(feature_group_old, dtype='float32')) #copy.deepcopy(features_old)
            H = cv.CreateMat(3,3,cv.CV_32F)
            
            new_object_count = 0
            #this code below is horrible
            while all_features.height >= 3:
                
                status = cv.CreateMat(1,all_features.height,cv.CV_8U)
                status = self.estimateRigidTransformationRANSAC(all_features_old, all_features_new)
                new_object = []
                j = 0
                print "status", numpy.asarray(status)
                
                all_features_np = numpy.asarray(all_features)
                all_features_old_np = numpy.asarray(all_features_old)
                
                status_np = numpy.asarray(status)
                print "len(status_np[0])",len(status_np[0])
                
                for i in range(len(status_np[0])):
                    if status_np[0,i]:
#                        print 'adding'
                        new_object.append(tuple(all_features_np[j,:]))
                        all_features_np = numpy.delete( all_features_np, j ,0)
                        all_features_old_np = numpy.delete( all_features_old_np, j ,0)
                    else:
                        j += 1
                if len(new_object) == 0:
                    print "should never get here"
                    break
                
#                print "new_object",new_object
                
                if new_object_count == 0: #we are still in the predominat osiiotn
                    new_idx = obj_idx
                else:
                    new_idx = object_count + new_object_count
                
                feature_sets_new.extend(zip(new_object, [new_idx] * len(new_object)) )
                    
                print "found new object numero new_idx",new_idx
#                print "feature_sets_new",feature_sets_new
                    
                new_object_count += 1
                
                
                all_features = cv.fromarray(all_features_np)
                all_features_old = cv.fromarray(all_features_old_np)
                
                
                co = cv.RGB(*col[new_idx])
                print "co",co
                for the_point in new_object:
                    cv.Circle (image, (int(the_point[0]), int(the_point[1])), 3,co, -1, 8, 0)
    
                    
                 
                
   
            
            
            
            
            
            
        
        
        
#        while all_features.height >= 4:
#            status = cv.CreateMat(1,all_features.height,cv.CV_8U)
#            cv.FindHomography(all_features,all_features_old,H,cv.CV_RANSAC, 3.0 , status = status)
#            new_object = []
#            j = 0
#            
#            all_features_np = numpy.asarray(all_features)
#            all_features_old_np = numpy.asarray(all_features_old)
#            
#            for i in range(status.width):
#                if status[0,i]:
#                    new_object.append(all_features_np[j,:])
#                    all_features_np = numpy.delete( all_features_np, j ,0)
#                    all_features_old_np = numpy.delete( all_features_old_np, j ,0)
#                else:
#                    j += 1
#            if len(new_object) == 0:
#                break
#            feature_sets_new.append(new_object)
#            
#            all_features = cv.fromarray(all_features_np)
#            all_features_old = cv.fromarray(all_features_old_np)
#            
#            col = (numpy.random.randint(256),numpy.random.randint(256),numpy.random.randint(256),0)
#            
#            for the_point in new_object:
#                cv.Circle (image, (int(the_point[0]), int(the_point[1])), 3, col, -1, 8, 0)
                
#        cv.ShowImage('LkDemo', image)
#        cv.WaitKey()
                
    
            
            # swapping
        print "Swapping"
        self.features_with_obj_idx = feature_sets_new
        self.prev_grey, self.grey = grey, prev_grey
        self.prev_pyramid, self.pyramid = pyramid, prev_pyramid
        need_to_init = False
        self.image = image
        return (features, feature_sets_new)
                
                
if __name__ == '__main__':
    
    frames = sys.argv[1:]
    if frames == []:
        print "usage lkdemo.py <image files>"
        sys.exit(1)
    
    ft = FeatureTracker()
    
    fc = 0
    is_inited = False
    
    while 1: 
        print "waiting for key press"
        # handle events
        c = cv.WaitKey (0)%0x100
    
        print "c: ", c%0x100
        if c == 113:
            # user has press the q key, so exit
            break
        
        print fc, len(frames)
        if fc == len(frames)-1:
            break
    
        # processing depending on the character
        if 32 <= c and c < 128:
            cc = chr(c).lower()
            if cc == 'r':
                need_to_init = True
                ft.initFirstFrame(cv.LoadImage(frames[fc]))
                is_inited = True
                fc = (fc + 10) % len(frames)
            elif cc == 'c':
                features = []
            elif cc == 'n':
                night_mode = not night_mode
            elif cc == ' ':
                fc = (fc + 1) % len(frames)
                if is_inited:
                    ft.track(cv.LoadImage(frames[fc]))
  
                
        if is_inited:     
            cv.ShowImage ('LkDemo', ft.image)
        else:
            cv.ShowImage('LkDemo', cv.LoadImage(frames[fc]) )
#        cv.WaitKey()
                
