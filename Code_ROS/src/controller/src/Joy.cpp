#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <string>
#include <std_msgs/Int8.h>
#include <time.h>
#include <sensor_msgs/Joy.h>

class Joy{
public:
	Joy(std::string name_of_topic_joy, std::string name_of_topic_joy_value, std::string name_of_topic_cmd);
  void get_joy_callback(sensor_msgs::Joy joy);

private:
	ros::NodeHandle node_; //Ros node
  ros::Publisher joy_value_pub_, cmd_pub_; //Ros publishers
  ros::Subscriber joy_sub_; //Ros subscriber
	int joy_value_;
  //joy_value_ = 0 for controller, joy_value_ = 1 for autonomous exploration, joy_value_ = 2 for going back to the start point
};

Joy::Joy(std::string name_of_topic_joy, std::string name_of_topic_joy_value, std::string name_of_topic_cmd){
	//name_of_topic_joy = name of the topic where the controller data is subscribed from
  //name_of_topic_joy_value = name of the topic where the joy value is published
  //name_of_topic_cmd = name of the topic where the cmd data is published
	joy_value_ = 0;
  std::cout<<"joy_value_ : "<<joy_value_<<std::endl;
	joy_value_pub_ = node_.advertise<std_msgs::Int8>(name_of_topic_joy_value,1);
  cmd_pub_ = node_.advertise<geometry_msgs::Twist>(name_of_topic_cmd,1);
  joy_sub_ = node_.subscribe(name_of_topic_joy, 1, &Joy::get_joy_callback, this);
}

void Joy::get_joy_callback(sensor_msgs::Joy joy){
//Gets the button value of the robot for its mode, and the joysticks values if the robot is to be commanded through the controller
  if(joy.buttons[1]==1){ //If user pushes "a", exploration mode
    joy_value_ = 1;
  }else if(joy.buttons[3]==1){ //If user pushes "x", getting back to initial position mode
    if(joy_value_ != 2){
      joy_value_ = 2;
    }
    else{
      joy_value_ = -1;
    }
  }else if(joy.buttons[2]==1){ //If user pushes "b", back to controller mode
    joy_value_ = 0;
  }
  // Control of robot through joysticks
  if(joy_value_==0 || joy_value_==1){
    float palier = 0.1, coupure = 0.07, angle;
    int compteur;
    geometry_msgs::Twist data_controller;
    data_controller.linear.x = joy.axes[1];
    data_controller.linear.y = joy.axes[0];
    angle = joy.axes[3];

    if(angle > 3.14){
      angle = angle- 3.14;
    }
    if(angle<-3.14){
      angle = angle + 3.14;
    }

    data_controller.angular.z = angle;

    cmd_pub_.publish(data_controller);
  }

  std_msgs::Int8 publishable;
  publishable.data = joy_value_;

  joy_value_pub_.publish(publishable);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "controller_control");
  Joy* new_joy = new Joy("/joy", "/joy_value", "/cmd_verified");
  while(ros::ok()){
    ros::spinOnce();
  }  	
  return 0;
}
