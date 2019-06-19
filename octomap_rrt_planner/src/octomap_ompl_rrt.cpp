//  June/2019, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "octomap_ompl_rrt/octomap_ompl_rrt.h"

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

void OctomapOmplRrt::cmdloopCallback(const ros::TimerEvent& event){

}

void OctomapOmplRrt::statusloopCallback(const ros::TimerEvent& event){

}