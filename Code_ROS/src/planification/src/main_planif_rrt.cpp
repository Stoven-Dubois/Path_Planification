#include <ros/ros.h>
#include <ros/package.h>
#include "Planif_rrt.cpp"

int main(int argc, char **argv){
  ros::init(argc, argv, "planification_through_rrt_algorithm");
  Planif_rrt* new_map = new Planif_rrt("/reduced_map", "/origin_robot", "/destination_robot", "/joy_value", "/path_rrt");

  	while(ros::ok()){
    	ros::spinOnce();
  	}  	
  return 0;
}
