#ifndef DRONE_CONTROLLER_H
#define DRONE_CONTROLLER_H

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h> // mavros_msgs::State
#include <boost/thread/thread.hpp>
#include <string>
#include <sensor_msgs/NavSatFix.h> // sensor_msgs::NavSatFix
#include <mavros_msgs/Altitude.h> // mavros_msgs::Altitude
#include <mavros_msgs/WaypointList.h> // mavros_msgs::WaypointList
#include <std_msgs/Float64.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/AttitudeTarget.h>
#include "tf/tf.h"
#include "markerpose.h"
#include "pid.h"
#include "angles/angles.h"
#include <numeric>
#include <math.h>
#include <list>
#include <fstream>

#define tracking_altitude_ 20
#define max_pixel_error_before_descending_ 50
#define max_angle_error_before_descending_ M_PI/6

class PID;

using namespace std;

class DroneController{

public:
  DroneController();
  void initialize();
  void update_drone_position();
  double get_moving_average(double input);
  vector<double> local_to_global_frame(double a, double x, double y);
  double add_angles(double angle1, double angle2);
  void print_data(double x_err, double y_err, double height);

  void state_cb(const mavros_msgs::State::ConstPtr& msg);
  void pose_cb(const drone_controller::markerpose::ConstPtr& msg);
  void altitude_cb(const mavros_msgs::Altitude::ConstPtr& msg);
  void heading_cb(const std_msgs::Float64::ConstPtr& msg);

  double get_gsd(double height, double fov, double pixel_w);
  double calculate_distance_to_target(double gsd, double x, double y);
  ~DroneController();
private:
ofstream save_data_file;
std_msgs::Float64 heading;
ros::NodeHandle nh;
ros::ServiceClient set_mode_client;
ros::Subscriber state_sub;
ros::Publisher local_pos_pub;
ros::ServiceClient arming_client;
ros::Subscriber point_sub;
ros::Subscriber alt_pos;
ros::Publisher target_velocity;
ros::Subscriber gps_heading;

mavros_msgs::State current_state;
mavros_msgs::Altitude altitude;
drone_controller::markerpose markerpose;

std::list<double> average_theta;

PID pid_position_x;
PID pid_position_y;
PID pid_yaw;
PID pid_height;

double tracking_altitude = tracking_altitude_;
double max_pixel_error_before_descending = max_pixel_error_before_descending_;
double max_angle_error_before_descending = max_angle_error_before_descending_;

double fov_camera;
double pixel_w_camera;

};

#endif
