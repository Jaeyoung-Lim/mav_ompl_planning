//  June/2019, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "octomap_rrt_planner/octomap_rrt_planner.h"
#include "octomap_rrt_planner/octomap_ompl_rrt.h"

using namespace Eigen;
using namespace std;
namespace ob = ompl::base;
namespace og = ompl::geometric;

//Constructor
OctomapRrtPlanner::OctomapRrtPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private),
  rrt_planner_(nh, nh_private) {

  cmdloop_timer_ = nh_.createTimer(ros::Duration(0.01), &OctomapRrtPlanner::cmdloopCallback, this); // Define timer for constant loop rate
  statusloop_timer_ = nh_.createTimer(ros::Duration(1), &OctomapRrtPlanner::statusloopCallback, this); // Define timer for constant loop rate

  Eigen::Vector3d lower, upper;
  lower << 0.0, 0.0, 0.0;
  upper << 1.0, 1.0, 1.0;

  rrt_planner_.setBounds(lower, upper);
  rrt_planner_.setupProblem();

}
OctomapRrtPlanner::~OctomapRrtPlanner() {
  //Destructor
}

void OctomapRrtPlanner::cmdloopCallback(const ros::TimerEvent& event){

}

void OctomapRrtPlanner::statusloopCallback(const ros::TimerEvent& event){

}

bool OctomapRrtPlanner::isStateValid(const ob::State *state){
  return true;
}

void OctomapRrtPlanner::planWithSimpleSetup(){
  Eigen::Vector3d start, goal;
  start << 0.1, 0.1, 0.1;
  goal << 0.9, 0.9, 0.9;
  rrt_planner_.getPath(start, goal);
}
