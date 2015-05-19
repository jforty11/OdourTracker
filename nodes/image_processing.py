#!/usr/bin/env python
# image_processing.py is bare-bones subscriber, in Object Oriented form. 
# If something is publishing to /camera/image_mono, it receives that 
# published image and writes "image received". 
# To run, use roslaunch on camera.launch or <bagfile>.launch and then, 
# in another terminal, type "python image_processing.py"
# Used in Lab 5 of BE 107 at Caltech
# By Melissa Tanner, mmtanner@caltech.edu, April 2015
# Modified by Lev Krayzman, May 2015 for odour tracking

import rospy
import cv2, cv 
import numpy as np
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

MIN_DIST = 25

class Image_Processor:
    def __init__(self):
       self.image_source = "/camera/image_mono"
       self.cvbridge = CvBridge()
       self.counter = 0

       # Raw Image Subscriber
       self.image_sub = rospy.Subscriber(self.image_source,Image,self.image_callback)

    def image_callback(self, rosimg):
        #print "image recieved"
        self.counter +=1
        if self.counter%100 is 0:
            # Convert the image.
            try:
                 # might need to change to bgr for color cameras
                img = self.cvbridge.imgmsg_to_cv2(rosimg, 'passthrough')
            except CvBridgeError, e:
                rospy.logwarn ('Exception converting background image from ROS to opencv:  %s' % e)
                img = np.zeros((320,240))
            

            # Process image
            # Invert image
            img = 255 - img
            img2 = img # Copy
    
            # Threshold with adaptive thresholding
            thresh2 = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 19, 8)
    
            contours2, hierarchy2 = cv2.findContours(thresh2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    
            contour_mean = []
            for i, e in enumerate(contours2):
              cont_x=[]
              cont_y=[]
              for i2, val in enumerate(e):
                for i3, val2 in enumerate(val):
                  #print(e)
                  cont_x.append(val2[0])
                  cont_y.append(val2[1])
              cont_mean = [np.mean(cont_x), np.mean(cont_y)]
              # Only store sufficiently separated contours
              store = True
              for i4, cur in enumerate(contour_mean):
                if(dist(cont_mean, cur) < MIN_DIST):
                  store = False
              if store : 
                contour_mean.append(cont_mean) 
                cv2.circle(img2, tuple([int(i) for i in cont_mean]), 5, (255, 0,0))
    
    
            #print contours
            cv2.drawContours(img2, contours2, -1, (0, 255, 0), 3)
    
            #cv2.imshow('dis image doe', img)
            #cv2.imshow('dis inmage doeeee', img2)

            #Display
            cv2.namedWindow('image', cv2.WINDOW_NORMAL)
            cv2.imshow('image', img)
            cv2.waitKey(10)
            #cv2.destroyAllWindows()

            # Print info
            print "Number of flies/maggots found:%d\n" % len(contour_mean)
            # print(contour_mean)

# Find distance between two points
def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1]);


################################################################################
def main():
  image_processor = Image_Processor()
  try:
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()     
  except KeyboardInterrupt:
    print "Shutting down"
  cv.DestroyAllWindows()

################################################################################
if __name__ == '__main__':
    rospy.init_node('image_processor', anonymous=True)
    main()
