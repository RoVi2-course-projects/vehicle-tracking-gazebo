#!/usr/bin/env python
# Third-party libraries
import numpy as np
import rospy
from geometry_msgs.msg import Point


class GPSSubscriber(rospy.Subscriber):
    """
    Create a custom image Subscriber, overriding ROS native one.
    """
    def __init__(self, sub_topic):
        rospy.Subscriber.__init__(self, sub_topic, Point, self.callback,
                                  queue_size=1)
        self.heading = np.array([0, 0])
        self.last_point = None

    def callback(self, gps_data):
        new_point = np.array([gps_data.x, gps_data.y])
        if self.last_point is not None:
            self.heading = new_point - self.last_point
        self.last_point = new_point
        return


def main():
    # Init node and publisher object
    rospy.init_node("heading", anonymous=True)
    # Subscribers configuration
    gps_subscriber = GPSSubscriber("/marker_model/gps_position")
    pub_topic = "/bus_heading"
    publisher = rospy.Publisher(pub_topic, Point, queue_size=1)
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        heading_point = Point(gps_subscriber.heading[0],
                              gps_subscriber.heading[1])
        publisher.publish(heading_point)
        rate.sleep()

if __name__ == "__main__":
    main()
