#!/usr/bin/env python
# Third-party libraries
import numpy as np
import rospy
from geometry_msgs.msg import Point


class KalmanFilterNode():

    def __init__(self):
        rospy.Subscriber("location", Point, self.marker_callback)
        rospy.Subscriber("bus_model/gps_position_noised", Point, self.gps_callback)
        # Variance matrices
        self.var = 30
        self.gps_var = 10
        self.marker_var = 30
        # Drone predicted and corrected position
        self.position = np.zeros([3,1])
        # GPS and marker measurements
        self.gps_input = None
        self.marker_input = None

    def gps_callback(self, data):
        self.gps_input = [data.x, data.y, data.z]
        return

    def marker_callback(self, data):
        self.marker_input = [data.x, data.y, data.z]
        return

    def update(self):
        if self.gps_input is None:
            self.gps_input = self.position
            self.gps_var = 500
        else:
             self.gps_var = 10
        if self.marker_input is None:
            self.marker_input = self.position
            self.marker_var = 500
        else:
            self.marker_var = 30
        # 1st stage: Update predicted state and var with GPS data
        pred_state = np.dot(np.eye(3), self.gps_input)
        pred_var = self.var + self.gps_var
        # 2nd stage: Correct the value with the marker data
        kalman_gain = pred_var / (pred_var+self.marker_var)
        self.position = pred_state + kalman_gain*(self.marker_input-
                                                  pred_state)
        self.var = pred_var * (1-kalman_gain)
        return


def main():
    # Init node and publisher object
    rospy.init_node("kalman_filter", anonymous=True)
    pub_topic = "/position_estimate"
    publisher = rospy.Publisher(pub_topic, Point, queue_size=1)
    est_position = Point
    # Instantiate a kalman object
    kalman_filter = KalmanFilterNode()
    # Run main loop
    rate = rospy.Rate(100)
    # while not rospy.is_shutdown():
    kalman_filter.update()
    est_position.x = kalman_filter.position[0][0]
    est_position.y = kalman_filter.position[1][0]
    est_position.z = kalman_filter.position[2][0]
    pos = kalman_filter.position
    print("Estimated pos: [{}, {}, {}]".format(est_position.x,
                                                est_position.y,
                                                est_position.z))
    publisher.publish(est_position)
    rate.sleep()
    return kalman_filter


if __name__ == "__main__":
    a = main()
