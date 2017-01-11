#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <string>
#include <std_msgs/Int8.h>
#include <time.h>

#define TIME_FOR_STARTING_OF_AUTONOMOUS_EXPLORATION_IN_SECONDS 0
#define TIME_FOR_STARTING_OF_RETURNING_TO_ORIGINAL_POINT_IN_SECONDS 22

class Joy_test{
public:
	Joy_test(std::string name_of_topic_joy);

private:
	ros::NodeHandle node_;
    ros::Publisher joy_pub_;
	int joy_value_;
};

Joy_test::Joy_test(std::string name_of_topic_joy){
	//name_of_topic_joy = name of the topic where the joy value is published
	joy_value_ = 0;
	joy_pub_ = node_.advertise<std_msgs::Int8>(name_of_topic_joy,1);

	clock_t t_begin = clock(), t_actual = clock();
	float diff = (float)((t_actual-t_begin)/CLOCKS_PER_SEC);

	std_msgs::Int8 publishable;

	while(ros::ok()){
		ros::spinOnce();
		t_actual = clock();
      	diff = (float)((t_actual-t_begin)/CLOCKS_PER_SEC);
      	if(diff >= TIME_FOR_STARTING_OF_AUTONOMOUS_EXPLORATION_IN_SECONDS){
      		if(diff < TIME_FOR_STARTING_OF_RETURNING_TO_ORIGINAL_POINT_IN_SECONDS){
      			joy_value_ = 1;
      		}else{
      			joy_value_ = 2;
      		}
      	}
      	publishable.data = joy_value_;
      	joy_pub_.publish(publishable);
	}
}

int main(int argc, char **argv){
  ros::init(argc, argv, "controller_test_through_gazebo");
  Joy_test* new_joy_test = new Joy_test("/joy_value");
  while(ros::ok()){
    ros::spinOnce();
  }  	
  return 0;
}
