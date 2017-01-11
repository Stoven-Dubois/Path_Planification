#include <ros/ros.h>
#include <ros/package.h>
#include <math.h>
#include <string>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

#define SPEED_LINEAR_MAX 1 //Maximum linear speed
#define SPEED_ROTATION_MAX 1 //Maximum rotative speed
#define LASER_DISTANCE 0.1 //Distance at which the robot is considered too near to the wall for allowing it to move forward

class Speed_control{
public:
	Speed_control(std::string name_topic_cmd_received, std::string name_topic_laser_1, std::string name_topic_laser_2, std::string name_topic_laser_3, std::string name_topic_laser_4, std::string name_topic_laser_5, std::string name_topic_cmd_sent);
	void get_cmd_callback(geometry_msgs::Twist cmd_received);
	void get_laser_1_callback(sensor_msgs::LaserScan laser);
  	void get_laser_2_callback(sensor_msgs::LaserScan laser);
  	void get_laser_3_callback(sensor_msgs::LaserScan laser);
  	void get_laser_4_callback(sensor_msgs::LaserScan laser);
  	void get_laser_5_callback(sensor_msgs::LaserScan laser);

private:
	ros::NodeHandle node_; //Ros node
	ros::Publisher cmd_pub_; //Ros publisher
    ros::Subscriber cmd_sub_, laser_1_sub_, laser_2_sub_, laser_3_sub_, laser_4_sub_, laser_5_sub_; //Ros subscribers
	float speed_linear_, speed_rotation_; //speeds sent to the robot
	float laser_1_, laser_2_, laser_3_, laser_4_, laser_5_; // laser speeds
};

Speed_control::Speed_control(std::string name_topic_cmd_received, std::string name_topic_laser_1, std::string name_topic_laser_2, std::string name_topic_laser_3, std::string name_topic_laser_4, std::string name_topic_laser_5, std::string name_topic_cmd_sent){
	speed_linear_ = 0;
	speed_rotation_ = 0;
	laser_1_sub_ = node_.subscribe(name_topic_laser_1, 1, &Speed_control::get_laser_1_callback, this);
  	laser_2_sub_ = node_.subscribe(name_topic_laser_2, 1, &Speed_control::get_laser_2_callback, this);
  	laser_3_sub_ = node_.subscribe(name_topic_laser_3, 1, &Speed_control::get_laser_3_callback, this);
  	laser_4_sub_ = node_.subscribe(name_topic_laser_4, 1, &Speed_control::get_laser_4_callback, this);
  	laser_5_sub_ = node_.subscribe(name_topic_laser_5, 1, &Speed_control::get_laser_5_callback, this);
  	cmd_sub_ = node_.subscribe(name_topic_cmd_received, 1, &Speed_control::get_cmd_callback, this);
	cmd_pub_ = node_.advertise<geometry_msgs::Twist>(name_topic_cmd_sent,1);
}

void Speed_control::get_cmd_callback(geometry_msgs::Twist cmd_received){
//Gets the demanded speeds for the robot and publish the resulting speeds
	speed_linear_ = cmd_received.linear.x;
	speed_rotation_ = cmd_received.angular.z;

	//if(laser_1_>=LASER_DISTANCE && laser_2_>=LASER_DISTANCE && laser_3_>=LASER_DISTANCE && laser_4_>=LASER_DISTANCE && laser_5_>=LASER_DISTANCE){
		if(speed_linear_ > SPEED_LINEAR_MAX){
			speed_linear_ = SPEED_LINEAR_MAX;
		}	
		if(speed_linear_ < - SPEED_LINEAR_MAX){
			speed_linear_ = -SPEED_LINEAR_MAX;
		}
	//}else{
	//	speed_linear_ = 0;
	//}
	if(speed_rotation_ > SPEED_ROTATION_MAX){
		speed_rotation_ = SPEED_ROTATION_MAX;
	}
	if(speed_rotation_ < -SPEED_ROTATION_MAX){
		speed_rotation_ = -SPEED_ROTATION_MAX;
	}

	geometry_msgs::Twist publishable;
    publishable.linear.x = speed_linear_;
    publishable.angular.z = speed_rotation_;
    cmd_pub_.publish(publishable);
}

void Speed_control::get_laser_1_callback(sensor_msgs::LaserScan laser){
//Gets the laser 1 data
  laser_1_ = laser.ranges[0];
}

void Speed_control::get_laser_2_callback(sensor_msgs::LaserScan laser){
//Gets the laser 1 data
  laser_2_ = laser.ranges[0];
}

void Speed_control::get_laser_3_callback(sensor_msgs::LaserScan laser){
//Gets the laser 1 data
  laser_3_ = laser.ranges[0];
}

void Speed_control::get_laser_4_callback(sensor_msgs::LaserScan laser){
//Gets the laser 1 data
  laser_4_ = laser.ranges[0];
}

void Speed_control::get_laser_5_callback(sensor_msgs::LaserScan laser){
//Gets the laser 1 data
  laser_5_ = laser.ranges[0];
}


int main(int argc, char **argv){
  ros::init(argc, argv, "speeds_controlling");
  Speed_control* new_speed_control = new Speed_control("/cmd_verified", "/IR1", "IR2", "IR3", "IR4", "IR5", "/cmd_vel");
  while(ros::ok()){
    ros::spinOnce();
  }  	
  return 0;
}

