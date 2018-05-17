#!/usr/bin/env python
# Third-party libraries
import numpy as numpy
import rospy
from geometry_msgs.msg import Point


def main():
    # Init node and publisher object
    rospy.init_node("gps_mock", anonymous=True)
    pub_topic = "/gps_location"
    publisher = rospy.Publisher(pub_topic, Point, queue_size=1)
    mock_point = Point(2,2,2)
    # Publish once per second
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        publisher.publish(mock_point)
        print("Publishing: [{}, {}, {}]".format(mock_point.x,
                                                mock_point.y,
                                                mock_point.z))
        rate.sleep()
    return


if __name__ == "__main__":
    main()
