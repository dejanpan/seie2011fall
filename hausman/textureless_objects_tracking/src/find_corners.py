#
#  Copyright (c) 2011, Robert Bosch LLC
#  All rights reserved.
# 
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions
#   are met:
# 
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above
#      copyright notice, this list of conditions and the following
#      disclaimer in the documentation and/or other materials provided
#      with the distribution.
#    * Neither the name of Robert Bosch LLC nor the names of its
#      contributors may be used to endorse or promote products derived
#      from this software without specific prior written permission.
# 
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#   POSSIBILITY OF SUCH DAMAGE.
# 
# 
#
#
#  \author Christian Bersch
#  

#! /usr/bin/env python

import roslib
roslib.load_manifest('textureless_objects_tracking')
import rospy

import numpy

import sys
import urllib

# import the necessary things for OpenCV
import cv
import cv_bridge
import geometry_msgs.msg
import textureless_objects_tracking.srv


class CornerFinder():

    def __init__(self):
        self.smooth_scale = 29
        self.eigval_threshold = 0.05
        self.max_ratio = 2.0
        self.convexity_thresh = 0.5
        self.check_step_radius = 15
        pass
    

                    
    def findCorners(self,src):
        dest = cv.CreateMat(src.height, 6 * src.width, cv.CV_32F)
        col = cv.CreateMat(src.height, src.width, cv.CV_8UC3)
        canny = cv.CreateMat(src.height, src.width, cv.CV_8U)
        canny_dil = cv.CreateMat(src.height, src.width, cv.CV_8U)
        cv.Canny(src, canny, 20, 80)
#to have just the good edges
        cv.Dilate(canny,canny_dil)
        
# create accumulator for the corners
        accumulator = cv.CreateMat(src.height, src.width, cv.CV_32F)
        cv.Zero(accumulator)
        cv.CvtColor(src, col, cv.CV_GRAY2BGR)
        cv.CornerEigenValsAndVecs(src, dest, self.smooth_scale, 5)
#get 6 dim matrix with eigenvectors and eigenvalus (lambda1 lambda2 x1 y1 x2 y2)
        a = numpy.asarray(dest)
        a = a.reshape((src.height, src.width, 6))
        for j in range(src.height):
            for i in range(src.width):
                if canny_dil[j,i] == 0:
                    continue
                
                lambda1 = abs(a[j,i,0])
                lambda2 = abs(a[j,i,1])

#check whether abs is a good aproximation

                if lambda1 > self.eigval_threshold and lambda2 > self.eigval_threshold:
                    #DEBUG
                    #if lambda1 - a[j,i,0] < -0.001 or \
                     #   lambda1 - a[j,i,0] > 0.001 or \
                      #  lambda2 - a[j,i,1] <  -0.001 or \
                       # lambda2 - a[j,i,1] >  0.001:
                        #print "sign!!!!!! %f %f %f %f" % (lambda1,a[j,i,0] ,lambda2,a[j,i,1] )
 
# if the corner is sharp enough      
                    if max(lambda1,lambda2)/min(lambda1,lambda2) < self.max_ratio and \
                     min(lambda1,lambda2)/max(lambda1,lambda2) > 1.0/self.max_ratio:
 
                        accumulator[j,i] = 1.0
# write every corner in accumulator 
#DEBUG
                        #print "acc", accumulator[j,i]
                        
                        
# smooth all the corners in accumulator and look for the best ones that don't differ after dilate               
        cv.Smooth(accumulator, accumulator, cv.CV_GAUSSIAN, self.smooth_scale, self.smooth_scale)
        
        accumulator_dil = cv.CreateMat(src.height, src.width, cv.CV_32F)
        cv.AddS(accumulator, 0.0, accumulator, canny)
        corners = []
        cv.Dilate(accumulator,accumulator_dil)
        for j in range(src.height):
            for i in range(src.width):
                if accumulator[j,i] > 0:
                    if accumulator[j,i] == accumulator_dil[j,i]:
                        corners.append((j,i))
                        
        
# create a ROI circle
        check_step_radius_img = cv.CreateMat(2*self.check_step_radius+2, 2*self.check_step_radius+2, cv.CV_8U)
        cv.Circle(check_step_radius_img,(self.check_step_radius+1, self.check_step_radius +1), self.check_step_radius, 255, -1)
        check_area = cv.Sum(check_step_radius_img)[0]
    
        cv.NamedWindow("check",1)
        convex_corners = []
        convex_corner_directions = []
        concave_corners = []
        concave_corner_directions = []
        
        col2 = cv.CloneMat(col)



	
        for (j,i) in corners:
# eigenvalues for the corners
            vec1 = a[j,i,2:4]
            vec2 = a[j,i,4:6]
            #DEBUG
#            print "l1 %f" % a[j,i,0]
#            print "vec1", vec1
#            print "l2 %f" % a[j,i,1]
#            print "vec1", vec2

            
            #check convexity
# with the AND operation between the ROI circle and the objects
            check_circle_img = cv.CreateMat(src.height, src.width, cv.CV_8U)
            cv.Zero(check_circle_img)
            cv.Circle(check_circle_img,(i, j), self.check_step_radius, 255, -1)
            check_circle_img_temp = cv.CreateMat(src.height, src.width, cv.CV_8U)
            cv.And(src, check_circle_img, check_circle_img_temp )
            check_area_this = cv.Sum(check_circle_img_temp)[0]
            #DEBUG
#            cv.ShowImage("check", check_circle_img_temp)
           # print "sum %d max %d" % (check_area_this, check_area)
#            cv.waitKey()
            
            if check_area_this/check_area > self.convexity_thresh:
                
                concave_corners.append((j,i))
                                #paint 
                cv.Circle(col, (i,j), 12, (0,255,0,0), 3)
                cv.Circle(col2, (i,j), 12, (0,255,0,0), 3)
                
                                #find direction
# for each direction draw a line pointing towards the corner or perpendicular to it
                check_line1_img = cv.CreateMat(src.height, src.width, cv.CV_8U)
                cv.Zero(check_line1_img)
                cv.Line(check_line1_img, (i,j),\
                                          tuple(numpy.array(self.check_step_radius * vec1 + (i,j), dtype = 'i')), 255, 1)
                cv.And(check_circle_img_temp, check_line1_img, check_line1_img )
# end of the 1st direction
                check_line1_len_pos = cv.Sum(check_line1_img)[0]
                cv.Zero(check_line1_img)
                cv.Line(check_line1_img, (i,j),\
                          tuple(numpy.array(self.check_step_radius * vec2 + (i,j), dtype = 'i')), 255, 1)
                cv.And(check_circle_img_temp, check_line1_img, check_line1_img )
                check_line2_len_pos = cv.Sum(check_line1_img)[0]
                
                cv.Zero(check_line1_img)
                cv.Line(check_line1_img, (i,j),\
                                          tuple(numpy.array(-self.check_step_radius * vec1 + (i,j), dtype = 'i')), 255, 1)
                cv.And(check_circle_img_temp, check_line1_img, check_line1_img )
                check_line1_len_neg = cv.Sum(check_line1_img)[0]
                cv.Zero(check_line1_img)
                cv.Line(check_line1_img, (i,j),\
                          tuple(numpy.array(-self.check_step_radius * vec2 + (i,j), dtype = 'i')), 255, 1)
                cv.And(check_circle_img_temp, check_line1_img, check_line1_img )
                check_line2_len_neg = cv.Sum(check_line1_img)[0]
                
            
# if the sum of two lines(one direction) is smaller than the other one, it means this is a good direction for concave corners 
                if check_line1_len_neg + check_line1_len_pos < check_line2_len_neg + check_line2_len_pos:
                    if check_line1_len_neg > check_line1_len_pos:
                        vec1 = -1.0 * vec1
#looking for the line from the direction
                    cv.Line(col, (i,j), tuple(numpy.array(20 * vec1 + (i,j), dtype = 'i')), (0,255,0), 3)
                    concave_corner_directions.append(vec1)
                    
                else:
                    if check_line2_len_neg > check_line2_len_pos:
                        vec2 = -1.0 * vec2
                   
                    cv.Line(col, (i,j), tuple(numpy.array(20 * vec2 + (i,j), dtype = 'i')), (0,255,0), 3)
                    concave_corner_directions.append(vec2)
                
                    
                    
                    
                    
                    
            else:
                convex_corners.append((j,i))
                
 
                #paint 
                cv.Circle(col, (i,j), 12, (0,0,255,0), 3)
                cv.Circle(col2, (i,j), 12, (0,0,255,0), 3)

                #find direction
                check_line1_img = cv.CreateMat(src.height, src.width, cv.CV_8U)
                cv.Zero(check_line1_img)
                cv.Line(check_line1_img, (i,j),\
                                          tuple(numpy.array(self.check_step_radius * vec1 + (i,j), dtype = 'i')), 255, 1)
                cv.And(check_circle_img_temp, check_line1_img, check_line1_img )
                check_line1_len_pos = cv.Sum(check_line1_img)[0]
                cv.Zero(check_line1_img)
                cv.Line(check_line1_img, (i,j),\
                          tuple(numpy.array(self.check_step_radius * vec2 + (i,j), dtype = 'i')), 255, 1)
                cv.And(check_circle_img_temp, check_line1_img, check_line1_img )
                check_line2_len_pos = cv.Sum(check_line1_img)[0]
                
                cv.Zero(check_line1_img)
                cv.Line(check_line1_img, (i,j),\
                                          tuple(numpy.array(-self.check_step_radius * vec1 + (i,j), dtype = 'i')), 255, 1)
                cv.And(check_circle_img_temp, check_line1_img, check_line1_img )
                check_line1_len_neg = cv.Sum(check_line1_img)[0]
                cv.Zero(check_line1_img)
                cv.Line(check_line1_img, (i,j),\
                          tuple(numpy.array(-self.check_step_radius * vec2 + (i,j), dtype = 'i')), 255, 1)
                cv.And(check_circle_img_temp, check_line1_img, check_line1_img )
                check_line2_len_neg = cv.Sum(check_line1_img)[0]
                
                
                if check_line1_len_neg + check_line1_len_pos < check_line2_len_neg + check_line2_len_pos:
                    if check_line2_len_neg > check_line2_len_pos:
                        vec2 = -1.0 * vec2
                    cv.Line(col, (i,j), tuple(numpy.array(20 * vec2 + (i,j), dtype = 'i')), (0,0,255), 3)
                    convex_corner_directions.append(vec2)
                    
                else:
                    if check_line1_len_neg > check_line1_len_pos:
                        vec1 = -1.0 * vec1
                    cv.Line(col, (i,j), tuple(numpy.array(20 * vec1 + (i,j), dtype = 'i')), (0,0,255), 3)
                    convex_corner_directions.append(vec1)
                    
                    
                
               

                
        
            
        cv.NamedWindow("corner_finder",1)
        cv.ShowImage("corner_finder", col)
        cv.WaitKey()
        cv.ShowImage("corner_finder", col2)
        cv.WaitKey()
        #DEBUG
	#cv.ShowImage("corner_finder", col3)
       # cv.WaitKey()
        
        
        return (col,accumulator,convex_corners, convex_corner_directions, concave_corners, concave_corner_directions)
    

        
    
    
    def findCornersService(self,req):
        br =  cv_bridge.CvBridge()
        iplImage = br.imgmsg_to_cv(req.image)
        (col,accumulator,convex_corners, convex_corner_directions, concave_corners, concave_corner_directions) = self.findCorners(iplImage)
        res = textureless_objects_tracking.srv.cornerFindResponse()
        for i in range(len (convex_corners)):
            pt =geometry_msgs.msg.Point()
            pt.x = convex_corners[i][1]
            pt.y = convex_corners[i][0]
            res.corner_convex.append(pt)
            pt = geometry_msgs.msg.Point32()
            pt.x = convex_corner_directions[i][0]
            pt.y = convex_corner_directions[i][1]
            res.push_direction_convex.append(pt)
            
        for i in range(len (concave_corners)):
            pt =geometry_msgs.msg.Point()
            pt.x = concave_corners[i][1]
            pt.y = concave_corners[i][0]
            res.corner.append(pt)
            pt = geometry_msgs.msg.Point32()
            pt.x = concave_corner_directions[i][0]
            pt.y = concave_corner_directions[i][1]
            res.push_direction.append(pt)
        return res
            
            
        
        
                        
                

        

if __name__ == '__main__':
    rospy.init_node('coner_finder_server')
    rospy.sleep(1.0)
    cf = CornerFinder()
    s = rospy.Service('find_corners',  textureless_objects_tracking.srv.cornerFind, lambda a: cf.findCornersService(a) )
    rospy.spin()

        

                
        
                
                
                
                
        
        
        
        

    

