#include <ros/ros.h>
#include <ros/package.h>
#include <math.h>
#include <string>
#include <opencv2/core/core.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int8.h>
#include <tf/transform_datatypes.h>

#define ROBOT_ENOUGH_CLOSE 1 //Distance at which the robot is considerered to have reached a point
#define RADIUS 0.25 //Radius of the robot
#define PI 3.14
#define U1_CONSTANT 0.5 //Linear speed of the robot when it is following the path

float max_speed_linear = 1; //Maximum linear speed
float max_speed_angular = 2; //Maximum rotative speed

class Trajectory_following{
public:
	Trajectory_following(std::string name_topic_joy, std::string name_topic_odometry, std::string name_topic_path, std::string name_topic_cmd);
	void get_odometry_callback(nav_msgs::Odometry odometry);
	void get_path_callback(nav_msgs::Path path);
	void get_joy_callback(std_msgs::Int8 joy_value);
	void get_trajectory_parameters();
	void get_correction_parameters();
	void send_speeds();
	int look_up_if_trajectory_point_near();

private:
	ros::NodeHandle node_; //Ros node
	ros::Publisher cmd_pub_; //Ros publisher
    ros::Subscriber odom_sub_, path_sub_, joy_sub_; //Ros subscribers
	float x_robot_, y_robot_, angle_robot_; // Odometry of the robot
	float speed_linear_, speed_rotation_; // Speeds sent to the robot
	float u1_, l1_, orientation_error_, linear_error_; // Parameters of the correction for the trajectory, using Frenet axes, and without caring for the orientation of the robot
	cv::Point2f* current_point_of_path_, * next_point_of_path_; //Points of the path whose the robot is in between
	std::vector<cv::Point2f*> path_; //Path followed by the robot
	int joy_value_; //joy_value_ = 0 for controller, joy_value_ = 1 for autonomous exploration, joy_value_ = 2 for going back to the start point
	int path_received_; // path_received_ = 1 if path_received, 0 else
};

Trajectory_following::Trajectory_following(std::string name_topic_joy, std::string name_topic_odometry, std::string name_topic_path, std::string name_topic_cmd){
//name_topic_joy = name of the topic where the joy value is published
//name_topic_odometry = name of the topic where the odometry is published
//name_topic_path = name of the topic where the path is published
//name_topic_cmd = name of the topic where the speed commands are published
	x_robot_ = 0; y_robot_ = 0, angle_robot_ = 0;
	speed_linear_ = 0; speed_rotation_ = 0;
	u1_ = U1_CONSTANT; l1_ = RADIUS;
	orientation_error_ = 0; linear_error_ = 0;
	joy_value_ = 0; path_received_ = 0;
	joy_sub_ = node_.subscribe(name_topic_joy, 1, &Trajectory_following::get_joy_callback, this);
	odom_sub_ = node_.subscribe(name_topic_odometry, 1, &Trajectory_following::get_odometry_callback, this);
	path_sub_ = node_.subscribe(name_topic_path, 1, &Trajectory_following::get_path_callback, this);
	cmd_pub_ = node_.advertise<geometry_msgs::Twist>(name_topic_cmd,1);

	int movement_finished = 0;

	while(ros::ok() && movement_finished == 0){
		ros::spinOnce();
		if(joy_value_ == 2){
			if(path_received_ == 1){
				get_correction_parameters();
				send_speeds();
				movement_finished = look_up_if_trajectory_point_near();
				if(movement_finished == 1){
					std::cout<<"Trajectory following finished"<<std::endl;
					path_received_ = 0;
				}
			}
		}
	}
	geometry_msgs::Twist publishable;
    publishable.linear.x = 0;
    publishable.angular.z = 0;
    cmd_pub_.publish(publishable);
}

void Trajectory_following::get_odometry_callback(nav_msgs::Odometry odom){
//Gets the odometry
	x_robot_ = odom.pose.pose.position.x;
	y_robot_ = odom.pose.pose.position.y;
	float mag = sqrt(odom.pose.pose.orientation.w*odom.pose.pose.orientation.w + odom.pose.pose.orientation.z*odom.pose.pose.orientation.z);
	angle_robot_ = tf::getYaw(odom.pose.pose.orientation);
	while(angle_robot_ > 2*PI){
		angle_robot_= angle_robot_ - (2*PI);
	}
	while(angle_robot_ < -2*PI){
		angle_robot_= angle_robot_ + (2*PI);
	}
	if(angle_robot_ > PI){
		angle_robot_ = angle_robot_ - (2*PI);
	}
	if(angle_robot_ < -PI){
		angle_robot_ = (angle_robot_ + (2*PI));
	}
}

void Trajectory_following::get_path_callback(nav_msgs::Path path){
//Gets the path
	if(path_received_ == 0 && joy_value_ == 2 && path.poses.size()>1){
		std::cout<<"receiving path"<<std::endl;
		path_received_ = 1;
		path_.clear();
		for(int i=0; i<path.poses.size();i++){
			cv::Point2f* new_point = new cv::Point2f(path.poses[i].pose.position.x, path.poses[i].pose.position.y);
			path_.push_back(new_point);
		}
		current_point_of_path_ = path_[0];
		next_point_of_path_ = path_[1];
	}
}

void Trajectory_following::get_joy_callback(std_msgs::Int8 joy_value){
//Gets the mode of the robot
	joy_value_ = joy_value.data;
}

void Trajectory_following::get_correction_parameters(){
//Gets the errors between the odometry of the robot and the one it should have according to the path
	//We calculate the angle between the x axis, and the trajectory line
	float theta_trajectory_line = atan2((next_point_of_path_->y - (current_point_of_path_->y)), (next_point_of_path_->x - current_point_of_path_->x));
	orientation_error_ = angle_robot_ - theta_trajectory_line;
	while(orientation_error_ > 2*PI){
		orientation_error_= orientation_error_ - (2*PI);
	}
	while(orientation_error_ < -2*PI){
		orientation_error_= orientation_error_ + (2*PI);
	}
	if(orientation_error_ > PI){
		orientation_error_ = orientation_error_ - (2*PI);
	}
	if(orientation_error_ < -PI){
		orientation_error_ = (orientation_error_ + (2*PI));
	}
	// We calculate y coordinate for the robot and a trajectory point, in an axis system where x is parallel to the trajectory, and y perpendiculer to the trajectory	
	float coordinate_y_of_trajectory_point_in_rotated_axis_system = (-1 * current_point_of_path_->x * sin(theta_trajectory_line)) + (current_point_of_path_->y * cos(theta_trajectory_line));
	float coordinate_y_of_robot_in_new_rotated_axis_system = (-1 * x_robot_ * sin(theta_trajectory_line)) + (y_robot_ * cos(theta_trajectory_line));
	linear_error_ = coordinate_yo_f_robot_in_new_rotated_axis_system - coordinate_y_of_trajectory_point_in_rotated_axis_system;
}

void Trajectory_following::send_speeds(){
//Calculates the linear and rotative speeds sent to the robot
	float kernel_function = 0.1*sin(orientation_error_)*exp(linear_error_);
	if(orientation_error_ < 0.79 && orientation_error_ > -0.79){
		speed_linear_ = u1_;
	}else{
		speed_linear_ = 0;
	}
	speed_rotation_ = ((-u1_)/(l1_)) * (sin(orientation_error_)) - (u1_)*kernel_function*linear_error_;
	geometry_msgs::Twist publishable;
    publishable.linear.x = speed_linear_;
    publishable.angular.z = speed_rotation_;
    cmd_pub_.publish(publishable);
}

int Trajectory_following::look_up_if_trajectory_point_near(){
//Updates the points of the path whose the robot is in between by looking if the robot has reached the next_point_of_path_
	int i=0, near_trajectory_point = 0;
	while(i<path_.size() && near_trajectory_point == 0){
		float distance = sqrt(pow(path_[i]->x - x_robot_,2)+pow(path_[i]->y - y_robot_,2));
		std::cout<<"robot : "<<x_robot_<<" "<<y_robot_<<std::endl;
		if(distance < ROBOT_ENOUGH_CLOSE){
			near_trajectory_point = 1;
			if(i == path_.size() - 1){
				return 1;
			}else{
				current_point_of_path_ = path_[i];
				next_point_of_path_ = path_[i+1];
				return 0;
			}
		}
		i++;
	}
	return 0;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "trajectory_following");
  Trajectory_following* new_trajectory_following = new Trajectory_following("/joy_value", "/odom", "/path_rrt", "/cmd_verified");
  while(ros::ok()){
    ros::spinOnce();
  }  	
  return 0;
}
