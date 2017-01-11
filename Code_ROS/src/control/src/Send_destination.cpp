#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include "control/PositionCart.h"
#include <string>
#include <std_msgs/Int8.h>

class Send_destination{
public:
	Send_destination(std::string name_topic_odometry, std::string name_topic_joy);
	void get_origin_callback(nav_msgs::Odometry odometry);
	void get_joy_callback(std_msgs::Int8 joy_value);

private:
	ros::NodeHandle node_; //ROS node
    ros::Publisher destination_pub_; //ROS publisher
    ros::Subscriber destination_sub_, joy_sub_; //ROS subscribers
	float x_destination_, y_destination_; //coordinates of the destination for the RRT algorithm
	int joy_value_; //joy_value_ = 0 for controller mode, joy_value_ = 1 for autonomous exploration, joy_value_ = 2 for going back to the start point
};

Send_destination::Send_destination(std::string name_topic_odometry, std::string name_topic_joy){ 
//name_topic_odometry = name of the topic where the odometry is published
//name_topic_joy = name of the topic where the joy value is published
	x_destination_ = 0;
	y_destination_ = 0;
	joy_sub_ = node_.subscribe(name_topic_joy, 1, &Send_destination::get_joy_callback, this);
	destination_sub_ = node_.subscribe(name_topic_odometry, 1, &Send_destination::get_origin_callback, this);
	destination_pub_ = node_.advertise<control::PositionCart>("destination_robot",1);
}

void Send_destination::get_origin_callback(nav_msgs::Odometry odometry){
//Gets the coordinates of the destination and publish them
	if(joy_value_ == 0){
		x_destination_ = odometry.pose.pose.position.x;
		y_destination_ = odometry.pose.pose.position.y;
	}
	control::PositionCart destination_to_publish;
	destination_to_publish.x = x_destination_;
	destination_to_publish.y = y_destination_;
	destination_pub_.publish(destination_to_publish);
}

void Send_destination::get_joy_callback(std_msgs::Int8 joy_value){
//Gets the mode of the robot
	joy_value_ = joy_value.data;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "sending_destination_coordinates");
  Send_destination* new_destination = new Send_destination("/odom", "/joy_value");
  	while(ros::ok()){
    	ros::spinOnce();
  	}  	
  return 0;
}
