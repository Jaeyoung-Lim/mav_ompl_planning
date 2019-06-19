//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#ifndef OCTOMAP_OMPL_RRT_H
#define OCTOMAP_OMPL_RRT_H

#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <tf/transform_broadcaster.h>

#include <stdio.h>
#include <cstdlib>
#include <string>
#include <sstream>

#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

class OctomapOmplRrt
{
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

  public:
    OctomapOmplRrt(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    virtual ~ OctomapOmplRrt();
};


#endif
