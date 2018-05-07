#include "drone_controller.h"

DroneController::DroneController(){
  ROS_INFO_STREAM("Dronecontroller constructor");
  initialize();
}

void DroneController::print_data(double x_err, double y_err, double height){
  cout << "current height: " << height<< endl;
  cout << "X: " << x_err << " Y: " << y_err << endl;
  cout << endl << "-----------------------------------------" << endl;

  save_data_file << ros::Time::now() << ";" << height << ";" << x_err << ";" << y_err << endl;
}

void DroneController::initialize(){
  ROS_INFO("Initializing subscribers, publishers, services and drone");
  save_data_file.open ("drone_data.txt");

  pid_yaw.init(0.1, 1.5, -1.5, 0.02, 0.5, 0.0);
  pid_position_x.init(0.1, 0.5, -0.5, 0.01, 0.0005, 0.0);
  pid_position_y.init(0.1, 0.5, -0.5, 0.01, 0.0005, 0.0);
  pid_height.init(0.1, 1.0, -1.0, 1.0, 0.1, 0.01);

  state_sub = nh.subscribe<mavros_msgs::State>
      ("mavros/state", 10, &DroneController::state_cb, this);
  local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
      ("mavros/setpoint_position/local", 10);
  arming_client = nh.serviceClient<mavros_msgs::CommandBool>
      ("mavros/cmd/arming");
  set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
      ("mavros/set_mode");
  point_sub = nh.subscribe<drone_controller::markerpose>
      ("/markerlocator/markerpose", 10, &DroneController::pose_cb, this);
  alt_pos = nh.subscribe<mavros_msgs::Altitude>
    ("mavros/altitude", 10, &DroneController::altitude_cb, this);
  target_velocity = nh.advertise<geometry_msgs::TwistStamped>
    ("mavros/setpoint_velocity/cmd_vel",10);
  gps_heading  = nh.subscribe<std_msgs::Float64>
      ("mavros/global_position/compass_hdg", 10, &DroneController::heading_cb,
      this);

  //the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);
  // wait for FCU connection
  while(ros::ok() && current_state.connected){
    ros::spinOnce();
    rate.sleep();
  }

  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  pose.pose.position.z = 1;
  //send a few setpoints before starting, or else you won't be able to
  //switch to offboard mode
  for(int i = 100; ros::ok() && i > 0; --i){
    local_pos_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
  }
  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;
  ros::Time last_request = ros::Time::now();

  if(current_state.mode != "OFFBOARD")
    if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
          ROS_INFO("Offboard enabled");


  if( !current_state.armed)
    if(arming_client.call(arm_cmd) && arm_cmd.response.success)
      ROS_INFO("Vehicle armed");
}

void DroneController::update_drone_position(){
  double head = angles::from_degrees(angles::normalize_angle(heading.data));

  double x_offset_from_center = markerpose.x;
  double y_offset_from_center = -markerpose.y;

  double angle = angles::from_degrees(heading.data);

  vector<double> rotated_pos = local_to_global_frame( heading.data,
                                                      x_offset_from_center,
                                                      y_offset_from_center);

  double pos_x_err = pid_position_x.calculate(0, -rotated_pos[0]);
  double pos_y_err = pid_position_y.calculate(0, -rotated_pos[1]);

  geometry_msgs::TwistStamped move_msg;

  double vel_z = pid_height.calculate(tracking_altitude, altitude.local);
  move_msg.twist.linear.z = vel_z;

  double avg = get_moving_average(markerpose.theta);
  double shortest_dist = angles::shortest_angular_distance( add_angles(avg,
                                                            -M_PI/2), head);
  double err = add_angles(M_PI, -abs(shortest_dist));
  double z_angular_velocity = pid_yaw.calculate(0, shortest_dist);
  cout << "err: " << abs(err) << " < " << max_angle_error_before_descending << endl;
  if (markerpose.quality > 0.5){
    move_msg.twist.linear.x = pos_x_err;
    move_msg.twist.linear.y = pos_y_err;
    move_msg.twist.angular.z = z_angular_velocity;

    if(abs(err) < max_angle_error_before_descending &&
        (abs(x_offset_from_center) + abs(y_offset_from_center)) < max_pixel_error_before_descending)
          if(tracking_altitude > 3.0){
            tracking_altitude -= 0.1;
            cout << "tracking_altitude: " << tracking_altitude << endl;
          }

  }
  print_data(x_offset_from_center, y_offset_from_center, altitude.local);
  target_velocity.publish(move_msg);

}

vector<double> DroneController::local_to_global_frame(double a, double x,
                                                      double y){
  double angle = angles::from_degrees(a);

  double rotationmatrix_inverse[2][2] = {{cos(angle), sin(angle)},
                                          {-sin(angle), cos(angle)}};
  vector<double> rotated_pos;
  rotated_pos.push_back(rotationmatrix_inverse[0][0] * x +
      rotationmatrix_inverse[0][1] * y);
  rotated_pos.push_back(rotationmatrix_inverse[1][0] * x +
      rotationmatrix_inverse[1][1] * y);
  return rotated_pos;
}

double DroneController::get_moving_average(double input){
   average_theta.push_back(input);
   if (average_theta.size() > 20) average_theta.pop_front();
     double sum = 0;
   for (std::list<double>::iterator p = average_theta.begin(); p !=
        average_theta.end(); ++p)
          sum += (double)*p;
   return sum / average_theta.size();
}

double DroneController::add_angles(double angle1, double angle2){
 double angle = angle1 + angle2;
 if(angle > 2.0*M_PI)
   angle -= M_PI;

 else if(angle < -0.0)
     angle += 2.0*M_PI;

 return angle;
}

void DroneController::state_cb(const mavros_msgs::State::ConstPtr& msg){
current_state = *msg;
}

void DroneController::pose_cb(const drone_controller::markerpose::ConstPtr&
      msg){
 markerpose = *msg;
}

void DroneController::altitude_cb(const mavros_msgs::Altitude::ConstPtr& msg){
 altitude = *msg;
}

void DroneController::heading_cb(const std_msgs::Float64::ConstPtr& msg){
 heading = *msg;
}

DroneController::~DroneController(){
    save_data_file.close();
}
