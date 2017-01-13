#include <ros/ros.h>
#include <ros/package.h>
#include <math.h>
#include <opencv2/core/core.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <random>
#include <time.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>
#include "map_edition/ReducedMap.h"
#include "Rrt_node.cpp"
#include "planification/PositionCart.h"
#include <std_msgs/Int8.h>

#define QUANTUM 50 //Distance in pixels between each node created and its father and sons
#define THRESHOLD_PIXELS 1 //Threshold used for separating map between obstacles and empty space

class Planif_rrt{
  public:
    Planif_rrt(std::string name_topic_reduced_map, std::string name_topic_origin, std::string name_topic_destination, std::string name_topic_joy, std::string name_topic_path_pub);
    void get_reduced_map_callback(map_edition::ReducedMap reduced_map);
    cv::Mat transform_from_reduced_map_to_mat(map_edition::ReducedMap);
    void get_origin_callback(planification::PositionCart origin);
    void get_destination_callback(planification::PositionCart destination);
    void get_joy_callback(std_msgs::Int8 joy_value);
    std::vector<cv::Point2i*> start(double delta_q, int nb_seconds);
    std::vector<cv::Point2i*> loop(double delta_q);
    std::vector<cv::Point2i*> loop_swaped(double delta_q);
    cv::Point2i* random_state();
    bool pixel_clear(int x, int y);
    Rrt_node* extend(Rrt_node* node, cv::Point2i* q_rand, double delta_q);
    cv::Point2i* calc_pos(cv::Point2i* near, cv::Point2i* q_rand, double delta_q);
    bool new_state(cv::Point2i* init, cv::Point2i* goal);
    bool near(cv::Point2i* init, cv::Point2i* goal, float delta_q);
    std::vector<cv::Point2i*> path(bool origin);
    std::vector<cv::Point2i*> path_partial(Rrt_node* q_new, int* size);
    void draw_rrt(cv::Mat map, std::vector<cv::Point2i*> path);
    std::vector<cv::Point2i*> post_traitement(std::vector<cv::Point2i*> path_found);
    nav_msgs::Path from_pixels_to_real_positions(std::vector<cv::Point2i*> path);
    void binarize(cv::Mat map_, float threshold);

  private:
    ros::NodeHandle node_; //Ros node
    ros::Publisher path_pub_; //Ros publisher
    ros::Subscriber map_sub_, origin_sub_, destination_sub_, joy_sub_; //Ros subscribers
    cv::Mat map_; // Reduced map (most little map containing all the pixels discovered)
    Rrt_node* origin_, * destination_, * fin_origine_, * fin_destination_;
    //origin_, destination_ = nodes used for growing the RRT tree
    //fin_origine_, fin_destination = nodes corresponding to the global start and end of the RRT algorithm
    float resolution_; //resolution of the map
    int state_, width_map_, height_map_, x_min_with_free_space_, y_min_with_free_space_, x_max_with_free_space_, y_max_with_free_space_;
    //state_ = state of the algorithm (-1 if blocked, 0 else)
    //height_map_, width_map_ = size of the reduced map
    //x_min_with_free_space_, y_min_with_free_space_, x_max_with_free_space_, y_max_with_free_space_ = parameters of the reduced map for its extremities
    bool inv_;
    //designing the part of the RRT tree growing (true = from the start, false = from the end)
    int joy_value_, initialisation_positions, nb_seconds_; 
    //joy_value_ = 0 for controller, joy_value_ = 1 for autonomous exploration, joy_value_ = 2 for going back to the start point
    //initialisation_positions = 0 if origin and destination not known for the robot, 1 else
    //nb_seconds = number of seconds given to the RRT algorithm for calculating the path
    int path_found_; //0 if not any path found, 1 if one found
    nav_msgs::Path path_; //path found
    cv::Point2f map_center_; //parameter of the reduced map for the center
};

Planif_rrt::Planif_rrt(std::string name_topic_reduced_map, std::string name_topic_origin, std::string name_topic_destination, std::string name_topic_joy, std::string name_topic_path_pub){ 
// name_topic_map = name of the topic where the ReducedMap is published
// name_topic_origin = name of the topic where the origin is published
// name_topic_destination = name of the topic where the destination is published
// name_topic_joy = name of the topic where the joy_value is published
// name_topic_path_pub = name of the topic where the path must be published
  resolution_ = 0;
  state_ = -2, width_map_ = 0, height_map_ = 0;
  initialisation_positions = 0;
  path_found_ = 0;
  inv_ = false;
  nb_seconds_ = 2;
  origin_ = new Rrt_node(new cv::Point2i(0,0));
  destination_ = new Rrt_node(new cv::Point2i(0,0));
  joy_sub_ = node_.subscribe(name_topic_joy, 1, &Planif_rrt::get_joy_callback, this);
	map_sub_ = node_.subscribe(name_topic_reduced_map, 1, &Planif_rrt::get_reduced_map_callback, this);
  origin_sub_ = node_.subscribe(name_topic_origin, 1, &Planif_rrt::get_origin_callback, this);
  destination_sub_ = node_.subscribe(name_topic_destination, 1, &Planif_rrt::get_destination_callback, this);
  path_pub_ = node_.advertise<nav_msgs::Path>(name_topic_path_pub,1);

  cv::namedWindow("MyWindow",CV_WINDOW_AUTOSIZE);
  ros::Rate loop_rate(100);
  cv::Size size(500,500);

  while(ros::ok()){
    ros::spinOnce();
    if(width_map_>0 && height_map_>0 && joy_value_ == 2){
      binarize(map_, THRESHOLD_PIXELS);
      std::vector<cv::Point2i*> path = start(QUANTUM, nb_seconds_);
      path = post_traitement(path);
      if(path_found_){
        path_ = from_pixels_to_real_positions(path);
      } 
      path_pub_.publish(path_);
      draw_rrt(map_, path);
      cv::imshow("MyWindow", map_);
      cv::waitKey(1000);
      loop_rate.sleep();
    }
  }
  cv::destroyWindow(("MyWindow"));
}

void Planif_rrt::get_origin_callback(planification::PositionCart origin){
//Gets the origin
  if(resolution_>0){
    int x = (origin.x - map_center_.x)/resolution_ - x_min_with_free_space_;
    int y = (-origin.y - map_center_.y)/resolution_ - y_min_with_free_space_;
    cv::Point2i* point = new cv::Point2i(x, y);
    origin_ = new Rrt_node(point);
  }
}

void Planif_rrt::get_destination_callback(planification::PositionCart destination){
//Gets the destination
  destination.x = 1;
  destination.y = -1;
  if(resolution_>0){
    int x = (destination.x - map_center_.x)/resolution_ - x_min_with_free_space_;
    int y = (-destination.y - map_center_.y)/resolution_ - y_min_with_free_space_;
	  cv::Point2i* point = new cv::Point2i(x, y);
    destination_ = new Rrt_node(point);
  }
}

void Planif_rrt::get_joy_callback(std_msgs::Int8 joy_value){
//Gets the mode
  joy_value_ = joy_value.data;
}


void Planif_rrt::get_reduced_map_callback(map_edition::ReducedMap reduced_map){ // Callback which gets the map, finds the efficient map, enlarges the obstacles and publishes a ReducedMap object
//Gets the reduced map
  resolution_ = reduced_map.resolution;

  map_center_.x = reduced_map.center_position_x;
  map_center_.y = reduced_map.center_position_y;

  x_min_with_free_space_ = reduced_map.x_min_with_free_space;
  y_min_with_free_space_ = reduced_map.y_min_with_free_space;
  x_max_with_free_space_ = reduced_map.x_max_with_free_space;
  y_max_with_free_space_ = reduced_map.y_max_with_free_space;

  width_map_ = x_max_with_free_space_ - x_min_with_free_space_ + 1;
	height_map_ = y_max_with_free_space_ - y_min_with_free_space_ + 1;
	
  map_ = transform_from_reduced_map_to_mat(reduced_map);
}

cv::Mat Planif_rrt::transform_from_reduced_map_to_mat(map_edition::ReducedMap reduced_map){ // Gets a nav_msgs::OccupancyGrid map and sends a cv::Mat map
//Transforms the reduced map into a Mat format
  cv::Mat mat(height_map_, width_map_, CV_8U);
	for(int j=0;j<height_map_;j++){
		for(int i=0;i<width_map_;i++){
			int indice = width_map_*j+i;
			mat.at<uchar>(j,i) = reduced_map.map[indice];
		}
	}
	return mat;
}

std::vector<cv::Point2i*> Planif_rrt::start(double delta_q, int nb_seconds){
//Start function of the RRT algorithm
  std::vector<cv::Point2i*> path;
  path_found_ = 1;
  clock_t t_begin = clock(), t_actual = clock();
  float diff = (float)((t_actual-t_begin)/CLOCKS_PER_SEC);
  while(diff < nb_seconds){
      path = loop(delta_q);
      inv_ = true;
      if(state_ == -1){
	state_ = 0;
      }
      if(state_ == 1){
	return path;
      }
      path = loop_swaped(delta_q);
      if(state_ == -1){
	state_ = 0;
      }
      if(state_ == 1){
	return path;
      }
      inv_ = false;
      t_actual = clock();
      diff = (double)((t_actual-t_begin)/CLOCKS_PER_SEC);
  }
  std::cout<<"No solution found in "<<nb_seconds<<" seconds"<<std::endl;
  path_found_ = 0;
}	

std::vector<cv::Point2i*> Planif_rrt::loop(double delta_q){
  cv::Point2i* q_rand = random_state();
  Rrt_node* q_new = extend(origin_, q_rand, delta_q);
  if(state_ != -1){
    Rrt_node* q_other_new = extend(destination_,q_new->get_pos(), delta_q);
    if(state_ == 1){
      fin_origine_ = q_new;
      fin_destination_ = q_other_new;
      return path(true); 
    }
  }
  std::vector<cv::Point2i*> out;
  out.push_back(q_rand);
  return out;
}

std::vector<cv::Point2i*> Planif_rrt::loop_swaped(double delta_q){
//Loop function from the end of the RRT algorithm
  cv::Point2i* q_rand = random_state();
  Rrt_node* q_new = extend(destination_, q_rand, delta_q);
  if(state_ != -1){
    Rrt_node* q_other_new = extend(origin_,q_new->get_pos(), delta_q);
    if(state_ == 1){
      fin_origine_ = q_other_new;
      fin_destination_ = q_new;
      return path(false); 
    }
  }
  std::vector<cv::Point2i*> out;
  out.push_back(q_rand);
  return out;
}

cv::Point2i* Planif_rrt::random_state(){
//Finds random point in the reduced map
  bool good_position_found = false;
  cv::Point2i* q_rand = new cv::Point2i(0,0);
  int x, y;
  std::random_device rd;
  std::uniform_int_distribution<int> distribution_x(0,width_map_);
  std::uniform_int_distribution<int> distribution_y(0,height_map_);
  while(!good_position_found){
    x = distribution_x(rd);
    y = distribution_y(rd);
    if(pixel_clear(x,y)){
      good_position_found = true;
      q_rand->x = x;
      q_rand->y = y;
    }
  }
  return q_rand;
}

bool Planif_rrt::pixel_clear(int x, int y){
//Returns true if corresponding pixel is empty, false else
  return map_.at<uchar>(y,x)<THRESHOLD_PIXELS;
}

Rrt_node* Planif_rrt::extend(Rrt_node* node, cv::Point2i* q_rand, double delta_q){
//Extend function of the RRT algorithm
  Rrt_node* near = node->nearest(node,q_rand);
  cv::Point2i* new_pose = calc_pos(near->get_pos(), q_rand, delta_q);
  Rrt_node* new_node = new Rrt_node(new_pose, near);
  if(!pixel_clear(new_pose->x,new_pose->y)){
    state_ = -1;
    return new_node;
  }
  bool test = Planif_rrt::new_state(near->get_pos(), new_pose);
  if (test){
    new_node = new Rrt_node(new_pose,near);
    near->add_next(new_node);
    if (Planif_rrt::near(new_node->get_pos(), q_rand, (float)delta_q)){
      state_ = 1;
      return new_node;
    }
    else{
      state_ = 0;
      return new_node;
    }
  }
  else{
    state_ = -1;
    return new_node;
  }
}

cv::Point2i* Planif_rrt::calc_pos(cv::Point2i* near, cv::Point2i* q_rand, double delta_q){
//Finds the position of the new node trying to reach the random point precedently created
  if(Rrt_node::dist(near,q_rand)<=(float)delta_q){
   return q_rand; 
  }
  float theta = atan2(q_rand->y-near->y,q_rand->x-near->x);
  cv::Point2i* res = new cv::Point2i(0,0);
  res->x = (int)(near->x + cos(theta)*delta_q);
  res->y = (int)(near->y + sin(theta)*delta_q);
  return res;
}

bool Planif_rrt::new_state(cv::Point2i* init, cv::Point2i* goal){
//Returns true if two points can be reachable with a line, false else
  cv::LineIterator line_it (map_, *init, *goal, 8);
  int cpt = line_it.count;
  for(int i=0;i<cpt;i++,++line_it){
    cv::Point p = line_it.pos();
    if(!pixel_clear(p.x, p.y)){
      return false;
    } 
  }
  return true;
}

bool Planif_rrt::near(cv::Point2i* init, cv::Point2i* goal, float delta_q){
//Return true if two points are near each other, false else
  return (Rrt_node::dist(init,goal) <= delta_q);
}

std::vector<cv::Point2i*> Planif_rrt::path(bool origin){
//Builds the path using the RRT trees
  std::vector<cv::Point2i*> tab_origin, tab_destination;
  int size_origin, size_destination;
  tab_origin = path_partial(fin_origine_, &size_origin);
  tab_destination = path_partial(fin_destination_, &size_destination);
  reverse(tab_origin.begin(), tab_origin.end());
  tab_origin.insert(tab_origin.end(), tab_destination.begin(), tab_destination.end());
  return tab_origin;
}

std::vector<cv::Point2i*> Planif_rrt::path_partial(Rrt_node* q_new, int* size){ 
//Builds the path of one tree
  int cpt = 0;
  std::vector<cv::Point2i*> tab;
  Rrt_node* q_current = q_new;
  while(q_current != NULL){
    tab.push_back(q_current->get_pos());
    q_current = q_current->get_previous();
  }
  *size = cpt;
  return tab;
}

void Planif_rrt::draw_rrt(cv::Mat map, std::vector<cv::Point2i*> path){
// Prints the RRT on the reduced map
  for(int i=0;i<path.size();i++){
    cv::Point2i* point = path.at(i);
    cv::circle(map, *point, 3, cv::Scalar(255,255,255), CV_FILLED, 8, 0);
    cv::circle(map, *point, 1, cv::Scalar(0,0,0), CV_FILLED, 8, 0);
    if(i>0){
      cv::line(map, *point, *path.at(i-1), cv::Scalar(255,255,255), 1, 8, 0);
    }
  }
}

std::vector<cv::Point2i*> Planif_rrt::post_traitement(std::vector<cv::Point2i*> path_found){
//Tries to find shrotcuts in the path found
  std::vector<cv::Point2i*> res;
  for(int i=0;i<path_found.size();i++){
    res.push_back(path_found[i]);
    int changed = 0;
    int j = path_found.size()-1;
    while(j>=i+2 && changed == 0){
      if(new_state(path_found[i], path_found[j])){
        std::cout << "found shortcut at point " << i << " towards " << j <<std::endl;
        changed = 1;
        i = j-1;
      }
      j = j-1;
    }
  }
  return res;
}

nav_msgs::Path Planif_rrt::from_pixels_to_real_positions(std::vector<cv::Point2i*> path){
//Transforms pixels coordinates into real coordinates
  nav_msgs::Path new_path;
  new_path.header.frame_id = "/map";
  for(int i=0;i<path.size();i++){
    float x = (path[i]->x + x_min_with_free_space_)*resolution_+map_center_.x;
    float y = (path[i]->y + y_min_with_free_space_)*resolution_+map_center_.y;
    geometry_msgs::PoseStamped new_pose;
    new_pose.pose.position.x = x;
    new_pose.pose.position.y = -y;
    new_pose.pose.position.z = 0;
    new_path.poses.push_back(new_pose);
  }
  return new_path;
}

void Planif_rrt::binarize(cv::Mat map, float threshold){
//Threshold separating the images between obstacles and empty space
  for(int i=0;i<map.cols;i++){
    for(int j=0;j<map.rows;j++){
      if(map.at<uchar>(j,i)<threshold){
        map.at<uchar>(j,i)=0;
      }else{
        map.at<uchar>(j,i)=255;
      }
    }
  }
}
