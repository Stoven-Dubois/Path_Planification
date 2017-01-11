#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include "control/PositionCart.h"
#include <string>
#include <std_msgs/Int8.h>

class Send_origin{
public:
	Send_origin(std::string name_topic_odometry, std::string name_topic_joy);
	void get_origin_callback(nav_msgs::Odometry odometry);
	void get_joy_callback(std_msgs::Int8 joy_value);
private:
	ros::NodeHandle node_; //ROS node
    ros::Publisher origin_pub_; //Ros publisher
    ros::Subscriber origin_sub_, joy_sub_; //Ros subscribers
	float x_origin_, y_origin_; //Coordinates of the origin for the RRT algorithm
	int joy_value_; //joy_value_ = 0 for controller, joy_value_ = 1 for autonomous exploration, joy_value_ = 2 for going back to the start point
};

Send_origin::Send_origin(std::string name_topic_odometry, std::string name_topic_joy){ 
//name_topic_odometry = name of the topic where the odometry is published
//name_topic_joy = name of the topic where the joy value is published
	x_origin_ = 0;
	y_origin_ = 0;
	joy_sub_ = node_.subscribe(name_topic_joy, 1, &Send_origin::get_joy_callback, this);
	origin_sub_ = node_.subscribe(name_topic_odometry, 1, &Send_origin::get_origin_callback, this);
	origin_pub_ = node_.advertise<control::PositionCart>("origin_robot",1);
}

void Send_origin::get_origin_callback(nav_msgs::Odometry odometry){
//Gets the coordinates of the origin and publish them
	x_origin_ = odometry.pose.pose.position.x;
	y_origin_ = odometry.pose.pose.position.y;
	control::PositionCart origin_to_publish;
	origin_to_publish.x = x_origin_;
	origin_to_publish.y = y_origin_;
	origin_pub_.publish(origin_to_publish);
}

void Send_origin::get_joy_callback(std_msgs::Int8 joy_value){
//Gets the mode of the robot
	joy_value_ = joy_value.data;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "sending_origin_coordinates");
  Send_origin* new_origin = new Send_origin("odom", "/joy_value");

  	while(ros::ok()){
    	ros::spinOnce();
  	}  	
  return 0;
}
