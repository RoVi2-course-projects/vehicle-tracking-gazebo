#!/usr/bin/env python
# Third-party libraries
import numpy as np
import rospy
from geometry_msgs.msg import Point
# Local libraries
import variance_model


class KalmanFilterNode():

    def __init__(self):
        # Subscribers configuration
        rospy.Subscriber("/markerlocator/location", Point, self.marker_callback,
                         queue_size=1)
        rospy.Subscriber("/marker_model/gps_position", Point, self.gps_callback,
                         queue_size=1)
        rospy.Subscriber("/marker_model/gps_position", Point, self.gps_callback_truth,
                         queue_size=1)
        # Publisher configuration
        pub_topic = "/position_estimate"
        self.publisher = rospy.Publisher(pub_topic, Point, queue_size=1)
        # Variance matrices
        self.var = 30
        self.gps_var = 10
        self.marker_var = 30
        # Drone predicted and corrected position
        self.position = np.zeros([3,1])
        # GPS and marker measurements
        self.gps_input_truth = None
        self.gps_input = None
        self.marker_input = None

        self.file_data = open("/home/per/GIT/logdata.txt","w")

    def gps_callback(self, data):
        self.gps_input = np.array([data.x, data.y, data.z]).reshape(3,1)
        return
    def gps_callback_truth(self, data):
        self.gps_input_truth = np.array([data.x, data.y, data.z]).reshape(3,1)
        return

    def marker_callback(self, data):
        self.marker_input = np.array([data.x, data.y, data.z]).reshape(3,1)
        return

    def update(self):
        marker_detected = True
        # Process the GPS data
        if self.gps_input_truth is None:
            self.gps_input_truth = [0,0,0]
        else:
            self.gps_input_truth = self.gps_input_truth


        if self.gps_input is None:
            self.gps_input = self.position
            self.gps_var = 500
            gps_err = 10000
            print("No GPS signal received.")

            self.file_data.write(str(gps_err))
            self.file_data.write(";")
        else:
            self.gps_var = 10
            gps_err = abs(self.gps_input[0])+abs(self.gps_input[1])
            self.file_data.write(str(gps_err)[1:-1])
            self.file_data.write(";")
            print("GPS data: [{}, {}, {}]".format(self.gps_input[0],
                                                  self.gps_input[1],
                                                  self.gps_input[2]))
        # Process the marker data
        if self.marker_input is None:
            marker_detected = False
            self.marker_input = self.position
            self.marker_var = 500
            print("No marker data received.")
            marker_err = 10000
            self.file_data.write(str(marker_err))
            self.file_data.write(";")
        else:
            marker_detected = True
            marker_err = abs(self.marker_input[0])+abs(self.marker_input[1])
            self.file_data.write(str(marker_err)[1:-1])
            self.file_data.write(";")
            distance = np.hypot(self.marker_input[0], self.marker_input[1])
            self.marker_var = variance_model.get_var_from_distance(distance)
            print("Marker data: [{}, {}, {}]".format(self.marker_input[0],
                                                     self.marker_input[1],
                                                     self.marker_input[2]))
        # 1st stage: Update predicted state and var with GPS data
        pred_state = np.dot(np.eye(3), self.gps_input)
        pred_var = self.var + self.gps_var
        # 2nd stage: Correct the value with the marker data
        kalman_gain = float(pred_var) / (pred_var+self.marker_var)
        self.position = pred_state + kalman_gain*(self.marker_input-
                                                  pred_state)
        self.var = pred_var * (1-kalman_gain)
        print("Estimated pos: [{}, {}, {}]".format(self.position[0],
                                                   self.position[1],
                                                   self.position[2]))
        self.gps_input = None
        self.marker_input = None

        pos_err = abs(self.position[0])+abs(self.position[1])
        self.file_data.write(str(pos_err)[1:-1])
        self.file_data.write(";")
        truth = abs(self.gps_input_truth[0])+abs(self.gps_input_truth[1])
        self.file_data.write(str(truth)[1:-1])
        self.file_data.write("\n")

        return

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.update()
            est_position = Point(self.position[0][0], self.position[1][0],
                                 self.position[2][0])
            self.publisher.publish(est_position)

            rate.sleep()
        return


def main():
    # Init node and publisher object
    rospy.init_node("kalman_filter", anonymous=True)
    # Instantiate the KalmaFilter class and run it.
    kalman_filter = KalmanFilterNode()
    kalman_filter.run()
    return


if __name__ == "__main__":
    a = main()
