#!/usr/bin/env python
from time import time, strftime
import os

# This code is modified from the MarkerLocator code from
# https://github.com/FroboLab/MarkerLocator
# https://github.com/henrikmidtiby/MarkerLocator

# sys.path.append('/opt/ros/indigo/lib/python2.7/dist-packages')
# need to run the following line before running the script in ros mode
# source /opt/ros/indigo/setup.bash

# python imports
import signal
import cv2
import math
import numpy as np

# application imports
from PerspectiveTransform import PerspectiveCorrecter
from MarkerPose import MarkerPose
from MarkerTracker import MarkerTracker

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# parameters
print_debug_messages = False
show_image = True
list_of_markers_to_find = [4, 4]
get_images_to_flush_cam_buffer = 5
publish_to_ros = True
markerpose_ros_topic = '/markerlocator/markerpose'
markerlocator_sub_topic = '/iris/camera/image_raw'

# global variables
stop_flag = False

if publish_to_ros:
    import rospy
    from markerlocator.msg import markerpose
    markerpose_msg = markerpose()

# define ctrl-c handler
def signal_handler(signal, frame):
    global stop_flag
    stop_flag = True

# install ctrl-c handler
signal.signal(signal.SIGINT, signal_handler)

class LocatorDriver:
    """
    Purpose: capture images from a ROS Topic and delegate procesing of the
    images to a different class.
    """

    def __init__(self, marker_orders=[4], default_kernel_size=14,
                 scaling_parameter=2500, downscale_factor=1):
        # Initialize camera driver.
        # Open output window.
        if show_image is True:
            cv2.namedWindow('filterdemo', cv2.WINDOW_AUTOSIZE)

        # Storage for image processing.
        self.current_frame = None
        self.processed_frame = None
        self.running = True
        self.downscale_factor = downscale_factor
        self.cv_image = None

        # Storage for trackers.
        self.trackers = []
        self.old_locations = []

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(markerlocator_sub_topic, Image,
                                          self.callback)
        # self.image_sub = rospy.Subscriber("/markerlocator/image_raw", Image,
        #                                   self.callback)

        # Initialize trackers.
        for marker_order in marker_orders:
            temp = MarkerTracker(marker_order, default_kernel_size,
                                 scaling_parameter)
            self.trackers.append(temp)
            self.old_locations.append(MarkerPose(None, None, None, None, None))

    def callback(self,data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
        except CvBridgeError as e:
            print(e)

    def get_image_size(self):
        if self.cv_image.shape == ():
            witdh = 1
            height = 1
        else:
            witdh, height = self.cv_image.shape[:2]
        return witdh, height

    def get_image(self):
        self.current_frame = np.copy(self.cv_image)

    def process_frame(self):
        if self.current_frame.shape == ():
            blank_image = np.zeros((800,800,3), np.uint8) #dummy if no image
            self.current_frame = blank_image
        else:
            frame_gray = np.copy(self.current_frame)
        self.processed_frame = self.current_frame
        frame_gray = cv2.cvtColor(self.current_frame, cv2.COLOR_RGB2GRAY)
        # Locate all markers in image.
        # import pdb; pdb.set_trace()
        reduced_image = cv2.resize(frame_gray, (0, 0),
                                   fx=1.0/self.downscale_factor,
                                   fy=1.0 / self.downscale_factor)
        for k in range(len(self.trackers)):
            # Previous marker location is unknown, search in the entire image.
            self.current_frame = self.trackers[k].locate_marker(reduced_image)
            self.old_locations[k] = self.trackers[k].pose
            self.old_locations[k].scale_position(self.downscale_factor)

    def draw_detected_markers(self):
        for k in range(len(self.trackers)):
            xm = self.old_locations[k].x
            ym = self.old_locations[k].y
            orientation = self.old_locations[k].theta
            if self.old_locations[k].quality < 0.9:
                cv2.circle(self.processed_frame, (xm, ym), 4, (55, 55, 255), 1)
            else:
                cv2.circle(self.processed_frame, (xm, ym), 4, (55, 55, 255), 3)

            xm2 = int(xm + 50 * math.cos(orientation))
            ym2 = int(ym + 50 * math.sin(orientation))
            cv2.line(self.processed_frame, (xm, ym), (xm2, ym2), (255, 0, 0), 2)

    def show_processed_frame(self):
        if show_image is True:
            cv2.imshow('filterdemo', self.processed_frame)

    def reset_all_locations(self):
        # Reset all markers locations,
        # forcing a full search on the next iteration.
        for k in range(len(self.trackers)):
            self.old_locations[k] = MarkerPose(None, None, None, None, None)

    def handle_keyboard_events(self):
        if show_image is True:
            # Listen for keyboard events and take relevant actions.
            key = cv2.waitKey(100)
            # Discard higher order bit,
            # http://permalink.gmane.org/gmane.comp.lib.opencv.devel/410
            key = key & 0xff
            if key == 27:  # Esc
                self.running = False
            if key == 114:  # R
                print("Resetting")
                self.reset_all_locations()
            if key == 115:  # S
                # save image
                print("Saving image")
                filename = strftime("%Y-%m-%d %H-%M-%S")
                cv2.imwrite("output/%s.png" % filename, self.current_frame)

    def return_positions(self):
        # Return list of all marker locations.
        return self.old_locations

class RosPublisher:
    def __init__(self, markers, markerpose_ros_topic):
        # Instantiate ros publisher with information about the markers that
        # will be tracked.
        self.markers = markers
        self.markerpose_pub = rospy.Publisher(markerpose_ros_topic, markerpose,
                                              queue_size=0)
        rospy.init_node('MarkerLocator')

    def publish_marker_locations(self, locations, witdh, height):
        markerpose_msg.header.stamp = rospy.get_rostime()
        j = 0
        for i in self.markers:
            # print 'x%i %i  y%i %i  o%i %i' % (i, locations[j].x, i,
            #   locations[j].y, i, locations[j].theta)
            # ros function

            markerpose_msg.order = locations[j].order
            markerpose_msg.x = locations[j].x - height/2.0
            markerpose_msg.y = locations[j].y - witdh/2.0
            markerpose_msg.theta = locations[j].theta
            markerpose_msg.quality = locations[j].quality
            self.markerpose_pub.publish(markerpose_msg)
            j += 1

def main():

    if publish_to_ros:
        ros_publisher = RosPublisher(list_of_markers_to_find,
                                     markerpose_ros_topic)

    ld = LocatorDriver(list_of_markers_to_find, default_kernel_size=14,
                       scaling_parameter=1000, downscale_factor=2)
    # ld = ImageDriver(list_of_markers_to_find, defaultKernelSize = 21)
    t0 = time()

    # Calibration of setup in robolab, so that the
    # coordinates correspond to real world coordinates.
    reference_point_locations_in_image = [[1328, 340], [874, 346], [856, 756],
                                          [1300, 762]]
    reference_point_loc_in_world_coord = [[0, 0], [300, 0],
                                                      [300, 250], [0, 250]]
    persp_corr = PerspectiveCorrecter(reference_point_locations_in_image,
                                      reference_point_loc_in_world_coord)

    while ld.running and stop_flag is False:
        (t1, t0) = (t0, time())
        if print_debug_messages is True:
            print "time for one iteration: %f" % (t0 - t1)
        ld.get_image()
        ld.process_frame()
        ld.draw_detected_markers()
        ld.show_processed_frame()
        ld.handle_keyboard_events()
        y = ld.return_positions()
        if publish_to_ros:
            w, h = ld.get_image_size()
            ros_publisher.publish_marker_locations(y, w, h)
        else:
            for k in range(len(y)):
                try:
                    pose_corr = persp_corr.convertPose(y[k])
                    print("%8.3f %8.3f %8.3f %8.3f %s" % (pose_corr.x,
                                                          pose_corr.y,
                                                          pose_corr.theta,
                                                          pose_corr.quality,
                                                          pose_corr.order))
                except Exception as e:
                    print("%s" % e)

    print("Stopping")

main()
