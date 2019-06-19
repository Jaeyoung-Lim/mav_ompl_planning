//  June/2019, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "octomap_rrt_planner/octomap_rrt_planner.h"

using namespace Eigen;
using namespace std;
//Constructor
OctomapRrtPlanner::OctomapRrtPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private) {

  cmdloop_timer_ = nh_.createTimer(ros::Duration(0.01), &OctomapRrtPlanner::cmdloopCallback, this); // Define timer for constant loop rate
  statusloop_timer_ = nh_.createTimer(ros::Duration(1), &OctomapRrtPlanner::statusloopCallback, this); // Define timer for constant loop rate

}
OctomapRrtPlanner::~OctomapRrtPlanner() {
  //Destructor
}

void OctomapRrtPlanner::cmdloopCallback(const ros::TimerEvent& event){

}

void OctomapRrtPlanner::statusloopCallback(const ros::TimerEvent& event){

}