#include <ros/ros.h>
#include <ros/package.h>
#include <cstddef>
#include <math.h>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/core/core.hpp>


#define MAX_NEXT 100

class Rrt_node{
public:
  Rrt_node();
  Rrt_node(cv::Point2i* position);
  Rrt_node(cv::Point2i* position, Rrt_node *previous);
  void add_next(Rrt_node *next);
  cv::Point2i* get_pos();
  int get_nb_next();
  Rrt_node* get_previous();
  Rrt_node* next_node(int id);
  static float dist(cv::Point2i* init, cv::Point2i* goal);
  Rrt_node* nearest(Rrt_node* init, cv::Point2i* goal);
  void print_node(cv::Mat map);
  
private:
  Rrt_node* previous_; // Father
  cv::Point2i* position_; // Position
  Rrt_node* next_[MAX_NEXT]; //Sons
  int nb_next_; //Number of sons
};

Rrt_node::Rrt_node()
{
  previous_ = NULL;
  position_ = new cv::Point2i(0,0);
  nb_next_ = 0;
  for(int i=0;i<MAX_NEXT;i++){
    next_[i] = NULL;
  }
}

Rrt_node::Rrt_node(cv::Point2i* position)
{
  previous_ = NULL;
  position_ = position;
  nb_next_ = 0;
  for(int i=0;i<MAX_NEXT;i++){
    next_[i] = NULL;
  }
}

Rrt_node::Rrt_node(cv::Point2i* position, Rrt_node *previous)
{
  previous_ = previous;
  position_ = position;
  nb_next_ = 0;
  for(int i=0;i<MAX_NEXT;i++){
    next_[i] = NULL;
  }
}

cv::Point2i* Rrt_node::get_pos(){
// Getter for the position
 return position_; 
}

Rrt_node* Rrt_node::next_node(int id){ 
// Gets the next node if possible, return NULL if not ; id = indice of the next node we want to access
  if(id >= nb_next_){
      std::cout<<"failed adding next node"<<std::endl;
      return NULL;	
  }
  return next_[id];
}


int Rrt_node::get_nb_next(){ 
// Returns number of next nodes
 return nb_next_; 
}

Rrt_node* Rrt_node::get_previous(){ 
// Returns the previous node (if the node is a root, its previous node is NULL)
 return previous_; 
}

void Rrt_node::add_next(Rrt_node* next) 
// Adds a node as a next node for the current node ; next = next node we want to add
{ 
  if(this->nb_next_>=MAX_NEXT){
    std::cout<<"Error : adding a node when nb of next is already " << MAX_NEXT << std::endl;
    return;
  }
  this->next_[this->nb_next_] = next;
  this->nb_next_++;

}

float Rrt_node::dist(cv::Point2i* init, cv::Point2i* goal){ 
// Returns the euclidean distance between an init point, and a goal point
  float res;
  res = sqrt(pow(init->x-goal->x,2)+pow(init->y-goal->y,2));
  return res;
}

Rrt_node* Rrt_node::nearest(Rrt_node* init, cv::Point2i* goal){ 
// Returns the nearest node from a tree of nodes to a point ; init : root of the nodes tree, goal = point considered
  if(init->get_nb_next()==0){
    return init; 
  }
  Rrt_node* res = init;
  float dist_min = dist(res->get_pos(),goal);
  for(int i=0; i<res->get_nb_next(); i++){
    float new_dist = dist(nearest(res->next_node(i),goal)->get_pos(),goal);
    if(dist_min > new_dist){
      res = nearest(res->next_node(i),goal);
      dist_min = new_dist;
    }
  }
  return res;
}

void Rrt_node::print_node(cv::Mat map){
  cv::circle(map, *this->get_pos(), 3, cv::Scalar(0,0,0), CV_FILLED, 8, 0);
  cv::circle(map, *this->get_pos(), 1, cv::Scalar(255,255,255), CV_FILLED, 8, 0);
  if(this->previous_){
    cv::line(map, *this->get_pos(), *(this->previous_->get_pos()), cv::Scalar(0,0,0));
  }
  for(int i=0; i<this->nb_next_; i++){
    this->next_[i]->print_node(map);
  }
}