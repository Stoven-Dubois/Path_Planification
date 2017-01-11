#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Int8.h>
#include <string>
#include <opencv2/core/core.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include <math.h>
#include "map_edition/ReducedMap.h"

#define Radius_of_robot 1 //Radius of robot
#define Radius_of_noise 0.05 //Radius maximum of pixels zones that are considerer noise if rounded by empty points

class Mapping{
public: 
	Mapping(std::string name_topic_map, std::string name_topic_init_pose, std::string name_topic_reduced_map);
	cv::Mat get_map();
	void get_map_callback(nav_msgs::OccupancyGrid map); 
	cv::Mat transform_from_map_to_mat(nav_msgs::OccupancyGrid map); 
	void find_useful_pixels(cv::Mat map);
	void enlarge_the_obstacles(int reduce, int raise);
	map_edition::ReducedMap transform_from_current_objet_to_publishable_type();
private:
	ros::NodeHandle node_; //Ros node
    ros::Publisher map_pub_; //Ros publisher
    ros::Subscriber map_sub_, init_pose_sub_; //Ros subscribers
    cv::Mat map_, mat_; // Reduced map (most little map containing all the pixels discovered)
    int x_min_with_free_space_, y_min_with_free_space_, x_max_with_free_space_, y_max_with_free_space_; // Coordinates of the efficient map

    float resolution_; // Map resolution
    cv::Point2f map_center_; // Position IRL of the cell (0,0) in the original map
};

Mapping::Mapping(std::string name_topic_map, std::string name_topic_init_pose, std::string name_topic_reduced_map){ 
// name_topic_map = name of the topic where the map is published
// name_topic_init_pose = name of the topic where the initial pose is published
// name_topic_reduced_map = name of the topic where the reduced map will be published
	map_pub_ = node_.advertise<map_edition::ReducedMap>(name_topic_reduced_map,1);
	map_sub_ = node_.subscribe(name_topic_map, 1, &Mapping::get_map_callback, this);
}

cv::Mat Mapping::get_map(){
//Getter for the reduced map
	return map_;
}

void Mapping::get_map_callback(nav_msgs::OccupancyGrid map){ 
//Gets the map, finds the efficient map, enlarges the obstacles and publishes a ReducedMap object
	resolution_ = map.info.resolution;
	map_center_.x = map.info.origin.position.x;
	map_center_.y = map.info.origin.position.y;
	cv::Mat mat = transform_from_map_to_mat(map);
	mat_  = mat;
	find_useful_pixels(mat);
	int reduce = Radius_of_noise/(resolution_);
	int enlarge = Radius_of_robot/resolution_;
	enlarge_the_obstacles(reduce, enlarge);
	map_pub_.publish(transform_from_current_objet_to_publishable_type());
}

cv::Mat Mapping::transform_from_map_to_mat(nav_msgs::OccupancyGrid map){
//Gets a nav_msgs::OccupancyGrid map and sends a cv::Mat map
	float width = map.info.width, height = map.info.height;
	cv::Mat mat(width, height, CV_8U);
	for(int i=0;i<width;i++){
		for(int j=0;j<height;j++){
			mat.at<uchar>(width-i-1,j) = map.data[(height)*i+j];
		}
	}
	return mat;
}

void Mapping::find_useful_pixels(cv::Mat map){ 
//Puts the efficient map in map_, and finds its coordinates
	//We find the coordinates of the most little box with all the discovered points
	x_min_with_free_space_ = map.rows;
	x_max_with_free_space_= 0;
	y_min_with_free_space_= map.cols;
	y_max_with_free_space_=0;
  	for(int i=0;i<map.cols;i++){
    	for (int j=0;j<map.rows;j++){
      		if(map.at<uchar>(j,i)<250){
				if(i<x_min_with_free_space_){
	  				x_min_with_free_space_ = i;
				}
				if(i>x_max_with_free_space_){
	  				x_max_with_free_space_ = i;
				}
				if(j<y_min_with_free_space_){
	  				y_min_with_free_space_ = j;
				}
				if(j>y_max_with_free_space_){
	  				y_max_with_free_space_ = j;
				}
      		}
    	}
 	}
  	if(x_min_with_free_space_>=x_max_with_free_space_ || y_min_with_free_space_>=y_max_with_free_space_ || x_min_with_free_space_<0 || x_max_with_free_space_>map.cols || y_min_with_free_space_<0 || y_max_with_free_space_>map.rows){
    std::cout<<"Error when calling find_useful_pixels"<<std::endl;
    	x_min_with_free_space_ = -1;
    	x_max_with_free_space_ = -1;
    	y_min_with_free_space_ = -1;
    	y_max_with_free_space_ = -1;
  	}
  	//We put the efficient map in map_
  	cv::Mat mat(y_max_with_free_space_ - y_min_with_free_space_ + 1, x_max_with_free_space_ - x_min_with_free_space_ + 1, CV_8U);
  	for(int i=0;i<mat.cols;i++){
		for(int j=0;j<mat.rows;j++){
			mat.at<uchar>(j,i) = map.at<uchar>(y_min_with_free_space_ + j, x_min_with_free_space_ + i);
		}
	}
	map_ = mat;
}

void Mapping::enlarge_the_obstacles(int reduce, int raise){ 
//reduce = degree of reducing the obstacles (eliminating the noise), raise = degree of enlarging the obstacles
  //Erodes the map to eliminate noise, then dilate to enlarge the obstacles
  if(reduce>0){
  	cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(reduce, reduce));
  	cv::erode(map_, map_, element);
  }
  if(raise>0){
  	cv::Mat element2 = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(reduce+raise, reduce+raise));
  	cv::dilate(map_, map_, element2);
  }
}

map_edition::ReducedMap Mapping::transform_from_current_objet_to_publishable_type(){
//Gets all the reduced map parameters and publish them
	map_edition::ReducedMap publishable;
	publishable.resolution = resolution_;
	publishable.x_min_with_free_space = x_min_with_free_space_;
	publishable.y_min_with_free_space = y_min_with_free_space_;
	publishable.x_max_with_free_space = x_max_with_free_space_;
	publishable.y_max_with_free_space = y_max_with_free_space_;

	int width = publishable.x_max_with_free_space - publishable.x_min_with_free_space + 1;
	int height = publishable.y_max_with_free_space - publishable.y_min_with_free_space + 1;

	for(int j=0;j<height;j++){
		for(int i=0;i<width;i++){
			publishable.map.push_back(map_.at<uchar>(j,i));
		}
	}	
	publishable.center_position_x = map_center_.x;
	publishable.center_position_y = map_center_.y;
	return publishable;
}


int main(int argc, char **argv){
  ros::init(argc, argv, "map_editing");
  Mapping* new_map = new Mapping("/map", "/odom", "reduced_map");

  while(ros::ok()){
    	ros::spinOnce();
  }

  return 0;
}