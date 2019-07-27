//  June/2019, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "octomap_rrt_planner/octomap_rrt_planner.h"

using namespace Eigen;
using namespace std;
namespace ob = ompl::base;
namespace og = ompl::geometric;

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

bool OctomapRrtPlanner::isStateValid(const ob::State *state){
  return true;
}

void OctomapRrtPlanner::planWithSimpleSetup(){
  // construct the state space we are planning in
  auto space(std::make_shared<ob::SE3StateSpace>());
  //  Set space bounds
  ob::RealVectorBounds bounds(3);
  bounds.setLow(-1);
  bounds.setHigh(1);
  space->setBounds(bounds);

  // Setup state space
  og::SimpleSetup ss(space);

  // ss.setStateValidityChecker([](const ob::State *state) { return isStateValid(state); });
  // ss.setStateValidityChecker(isStateValid);

  // Set random goal states
  ob::ScopedState<> start(space);
  start.random();
  ob::ScopedState<> goal(space);
  goal.random();

  ss.setStartAndGoalStates(start, goal);

  ob::PlannerStatus solved = ss.solve(1.0);
  if (solved) {
        std::cout << "Found solution:" << std::endl;
        // print the path to screen
        ss.simplifySolution();
        ss.getSolutionPath().print(std::cout);
  } else {
    std::cout << "Solution Not found" << std::endl;
  }
}
