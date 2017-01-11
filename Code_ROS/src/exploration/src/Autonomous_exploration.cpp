#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <string>
#include <math.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <time.h>
#include <random>
#include <tf/transform_datatypes.h>


#define LINEAR_SPEED 1 
#define ROTATIVE_SPEED 1
#define LASER_DISTANCE 0.4
#define ROTATION_DISTANCE 0.1
#define PI 3.14

#define TIME_INITIAL_ROTATION 10
#define TIME_INITIAL_LINEAR 20
#define TIME_SECOND_ROTATION 30
#define TIME_LEFT_TURN_FINISHED 50
#define TIME_RIGHT_TURN_FINISHED 80


class Autonomous_exploration{
public:
  Autonomous_exploration(std::string name_topic_joy, std::string name_topic_odometry, std::string name_topic_laser_1, std::string name_topic_laser_2, std::string name_topic_laser_3, std::string name_topic_laser_4, std::string name_topic_laser_5, std::string name_topic_cmd);
	void get_joy_callback(std_msgs::Int8 joy_value);
  void get_laser_1_callback(sensor_msgs::LaserScan laser);
  void get_laser_2_callback(sensor_msgs::LaserScan laser);
  void get_laser_3_callback(sensor_msgs::LaserScan laser);
  void get_laser_4_callback(sensor_msgs::LaserScan laser);
  void get_laser_5_callback(sensor_msgs::LaserScan laser);
  void get_odometry_callback(nav_msgs::Odometry odometry);
  void do_rotation();
  void do_nearest_rotation();
  void exploration();

private:
	ros::NodeHandle node_;
  ros::Subscriber joy_sub_, odom_sub_, laser_1_sub_, laser_2_sub_, laser_3_sub_, laser_4_sub_, laser_5_sub_;
  ros::Publisher cmd_pub_;
  float orientation_robot_, speed_rotation_, rotation_limit_, speed_linear_, laser_1_, laser_2_, laser_3_, laser_4_, laser_5_;
	float orientation_robot_wanted_; //orientation wanted for the robot
  int movement_initiated_; //0 if robot going forward normally, 1 if avoiding an obstacle
  int joy_value_; //joy_value_ = 0 for controller, joy_value_ = 1 for autonomous exploration, joy_value_ = 2 for going back to the start point
  int sens_of_rotation_; //0 for no rotation, 1 for left rotation,, -1 for right rotation 
  int stopped_; //sending a stop movement at end of exploration mode
};

Autonomous_exploration::Autonomous_exploration(std::string name_topic_joy, std::string name_topic_odometry, std::string name_topic_laser_1, std::string name_topic_laser_2, std::string name_topic_laser_3, std::string name_topic_laser_4, std::string name_topic_laser_5, std::string name_topic_cmd){
  //name_topic_joy = name of the topic where the joy value is published
  //name_topic_odometry = name of the topic where the odometry is published
  //name_topic_laser = name of the topics where the infrared lasers are published
  //name_topic_cmd = name of the topic where the speed commands are published
  joy_value_ = 0; 
  sens_of_rotation_ = 0;
  orientation_robot_ = 0; rotation_limit_ = 0;
  speed_rotation_ = 0; speed_linear_ = 0;
  stopped_ = 0;
  laser_1_ = 0; laser_2_ = 0; laser_3_ = 0; laser_4_ = 0; laser_5_ = 0;
  laser_1_sub_ = node_.subscribe(name_topic_laser_1, 1, &Autonomous_exploration::get_laser_1_callback, this);
  laser_2_sub_ = node_.subscribe(name_topic_laser_2, 1, &Autonomous_exploration::get_laser_2_callback, this);
  laser_3_sub_ = node_.subscribe(name_topic_laser_3, 1, &Autonomous_exploration::get_laser_3_callback, this);
  laser_4_sub_ = node_.subscribe(name_topic_laser_4, 1, &Autonomous_exploration::get_laser_4_callback, this);
  laser_5_sub_ = node_.subscribe(name_topic_laser_5, 1, &Autonomous_exploration::get_laser_5_callback, this);
  odom_sub_ = node_.subscribe(name_topic_odometry, 1, &Autonomous_exploration::get_odometry_callback, this);
  joy_sub_ = node_.subscribe(name_topic_joy, 1, &Autonomous_exploration::get_joy_callback, this);
  cmd_pub_ = node_.advertise<geometry_msgs::Twist>(name_topic_cmd,1);
}

void Autonomous_exploration::get_laser_1_callback(sensor_msgs::LaserScan laser){
  laser_1_ = laser.ranges[0];
}

void Autonomous_exploration::get_laser_2_callback(sensor_msgs::LaserScan laser){
  laser_2_ = laser.ranges[0];
}

void Autonomous_exploration::get_laser_3_callback(sensor_msgs::LaserScan laser){
  laser_3_ = laser.ranges[0];
}

void Autonomous_exploration::get_laser_4_callback(sensor_msgs::LaserScan laser){
  laser_4_ = laser.ranges[0];
}

void Autonomous_exploration::get_laser_5_callback(sensor_msgs::LaserScan laser){
  laser_5_ = laser.ranges[0];
}

void Autonomous_exploration::get_joy_callback(std_msgs::Int8 joy_value){
  //Gets the controller data and publishes the speeds calculated for autonomous exploration
	joy_value_ = joy_value.data;

  if(joy_value_ == 1){
    exploration();
    geometry_msgs::Twist publishable;
    publishable.linear.x = speed_linear_;
    publishable.angular.z = speed_rotation_;
    cmd_pub_.publish(publishable);
  }else{
    if(stopped_ == 0){
      stopped_ = 1;
      geometry_msgs::Twist publishable;
      publishable.linear.x = 0;
      publishable.angular.z = 0;
      cmd_pub_.publish(publishable);
    }
  }
}

void Autonomous_exploration::get_odometry_callback(nav_msgs::Odometry odometry){
//Gets the odometry of the robot
  float mag = sqrt(pow(odometry.pose.pose.orientation.w,2) + pow(odometry.pose.pose.orientation.z,2));
  orientation_robot_ = tf::getYaw(odometry.pose.pose.orientation);

  while(orientation_robot_ > 2*PI){
    orientation_robot_= orientation_robot_ - (2*PI);
  }
  while(orientation_robot_ < -2*PI){
    orientation_robot_= orientation_robot_ + (2*PI);
  }
  if(orientation_robot_ > PI){
    orientation_robot_ = orientation_robot_ - (2*PI);
  }
  if(orientation_robot_ < -PI){
    orientation_robot_ = (orientation_robot_ + (2*PI));
  }
}

void Autonomous_exploration::do_rotation(){
//Do a full rotation of the robot specified by its odometry and desired angle attributes
  movement_initiated_ = 1;
  speed_linear_ = 0;
  float orientation_difference = orientation_robot_wanted_ - orientation_robot_;
  if(orientation_difference > 0){
    speed_rotation_ = ROTATIVE_SPEED;
  }else{
    speed_rotation_ = - ROTATIVE_SPEED;
  }
}

void Autonomous_exploration::do_nearest_rotation(){
//Finds the direction where the is the most little rotation to do to be able to go forward, and rotates the robot following this direction
  float value_left_sensors = laser_1_ + laser_2_;
  float value_right_sensors = laser_4_ + laser_5_;
  speed_linear_ = 0;
  if(value_left_sensors < value_right_sensors){
    speed_rotation_ = - ROTATIVE_SPEED;
  }else{
    speed_rotation_ = ROTATIVE_SPEED;
  }
}

void Autonomous_exploration::exploration(){
//If the robot can move forward, it moves forward
//If not, it rotates to a random angle
//If it can move forward again, it moves, if not, it makes the most little rotation to be able to move forward again, and moves forward
  if(movement_initiated_ == 0){
      if(laser_1_>=LASER_DISTANCE && laser_2_>=LASER_DISTANCE && laser_3_>=LASER_DISTANCE && laser_4_>=LASER_DISTANCE && laser_5_>=LASER_DISTANCE){
        speed_linear_ = LINEAR_SPEED;
        speed_rotation_ = 0;
      }else{
        std::random_device rd;
        std::uniform_int_distribution<int> distribution_angle(-180, 180);
        orientation_robot_wanted_ = distribution_angle(rd)*PI/180;
        do_rotation();
      }
  }else{
      if(movement_initiated_ == 2){
        if(laser_1_>=LASER_DISTANCE && laser_2_>=LASER_DISTANCE && laser_3_>=LASER_DISTANCE && laser_4_>=LASER_DISTANCE && laser_5_>=LASER_DISTANCE){
          movement_initiated_ = 0;
        }
      }else if(orientation_robot_ > orientation_robot_wanted_ - ROTATION_DISTANCE && orientation_robot_ < orientation_robot_wanted_ + ROTATION_DISTANCE){
        movement_initiated_ = 2;
        do_nearest_rotation();
      }
  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "exploration");
  Autonomous_exploration* new_exploration = new Autonomous_exploration("/joy_value", "/odom", "/IR1", "IR2", "IR3", "IR4", "IR5", "/cmd_verified");

  	while(ros::ok()){
    	ros::spinOnce();
  	}  	
  return 0;
}
