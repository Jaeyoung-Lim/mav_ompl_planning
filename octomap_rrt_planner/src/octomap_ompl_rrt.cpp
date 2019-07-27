//  June/2019, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "octomap_rrt_planner/octomap_ompl_rrt.h"

using namespace Eigen;
using namespace std;
//Constructor
OctomapOmplRrt::OctomapOmplRrt(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private) {

}
OctomapOmplRrt::~OctomapOmplRrt() {
  //Destructor
}

void OctomapOmplRrt::setupProblem(){

  problem_setup_.setOctomapCollisioNChecking();
  ompl::base::RealVectorBounds bounds(3);
  bounds.setLow(0, lower_bound_.x());
  bounds.setLow(1, lower_bound_.y());
  bounds.setLow(2, lower_bound_.z());

  bounds.setHigh(0, upper_bound_.x());
  bounds.setHigh(1, upper_bound_.y());
  bounds.setHigh(2, upper_bound_.z());

  // Define start and goal positions.
  problem_setup_.getGeometricComponentStateSpace()
      ->as<ompl::base::RealVectorStateSpace>()
      ->setBounds(bounds);
}

void OctomapOmplRrt::setBounds(Eigen::Vector3d& lower_bound, Eigen::Vector3d& upper_bound){
  lower_bound_ = lower_bound;
  upper_bound_ = upper_bound;

}

bool OctomapOmplRrt::getPath(const Eigen::Vector3d& start, const Eigen::Vector3d& goal){
  ompl::base::PlannerStatus solved = problem_setup_.solve(1.0);
  if (solved) {
        std::cout << "Found solution:" << std::endl;
        // print the path to screen
        problem_setup_.simplifySolution();
        problem_setup_.getSolutionPath().print(std::cout);
  } else {
    std::cout << "Solution Not found" << std::endl;
  }  
}