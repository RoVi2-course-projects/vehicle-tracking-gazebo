#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from geometry_msgs.msg import Point
import numpy as np

class LocateMarker:

  def __init__(self):
    # Publish to the MarkerLocator's original topic
    #self.image_pub = rospy.Publisher("/markerlocator/image_raw",Image)

    # publish point
    self.loc_pub = rospy.Publisher("location", Point, queue_size=10)
    self.last_cx = 0
    self.last_cy = 0
    # Subscribe from the Iris camera topic
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/iris/camera/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
    except CvBridgeError as e:
      print(e)

    small = cv2.resize(cv_image, (0,0), fx=0.5, fy=0.5)
    cv_image = small
    # threshold
    img_hsv = cv2.cvtColor(cv_image, cv2.COLOR_RGB2HSV)
    image_gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
    # lower mask (0-10)
    lower_red = np.array([0,40,40])
    upper_red = np.array([15,255,255])
    mask0 = cv2.inRange(img_hsv, lower_red, upper_red)

    # upper mask (170-180)
    lower_red = np.array([160,40,40])
    upper_red = np.array([180,255,255])
    mask1 = cv2.inRange(img_hsv, lower_red, upper_red)

    # join my masks
    mask = mask0+mask1
    # believe* is what you want
    image_gray[np.where(mask==0)] = 0
    # ret,thresh1 = cv2.threshold(frame_gray,100,255,cv2.)
    im2,contours,hierarchy = cv2.findContours(image_gray, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    witdh, height = cv_image.shape[:2]
    print(witdh, height)
    
    if contours:
        cnt = contours[0]
        M = cv2.moments(cnt)

        if not M['m00'] == 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            print(cx,cy)
        else:
            cx = self.last_cx
            cy = self.last_cy
        #print( M )
        cv2.circle(cv_image,(cx,cy), 20, (255,0,0), 4)
        point = Point(cy-height/2, cx-witdh/2, 0.0)

    else:
        point = Point(0,0,0)

    # Display the image
    #cv2.line(cv_image,(0,0),(511,511),(255,0,0),5)
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    self.last_cx = cx
    self.last_cy = cy

    #frame_gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)

    try:
      print("none")
      self.loc_pub.publish(point)
      #self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame_gray, "8UC1"))
    except CvBridgeError as e:
      print(e)

def main(args):

  rospy.init_node('image_converter', anonymous=True)
  ic = LocateMarker()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
