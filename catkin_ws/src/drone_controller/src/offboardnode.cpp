#include <ros/ros.h>
#include "drone_controller.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "offb_node");
  DroneController drone_controller;

  ros::Rate rate(20.0);
  while(ros::ok()){
    drone_controller.update_drone_position();
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
