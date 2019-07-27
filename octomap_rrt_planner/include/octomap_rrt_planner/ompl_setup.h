#ifndef OMPL_SETUP_H
#define OMPL_SETUP_H

#include <ompl/geometric/SimpleSetup.h>
#include "ompl/base/SpaceInformation.h"
#include <ompl/base/spaces/SE3StateSpace.h>
#include <octomap_rrt_planner/ompl_octomap.h>

namespace ompl {

class OmplSetup : public geometric::SimpleSetup {
  public:
    OmplSetup() : geometric::SimpleSetup(base::StateSpacePtr(new base::RealVectorStateSpace(3))) {}

    const base::StateSpacePtr& getGeometricComponentStateSpace() const {
      return getStateSpace();
    }

    void setOctomapCollisioNChecking(){
      std::shared_ptr<OctomapValidityChecker> validity_checker(new OctomapValidityChecker(getSpaceInformation()));

      setStateValidityChecker(base::StateValidityCheckerPtr(validity_checker));
      si_->setMotionValidator(base::MotionValidatorPtr(new OctomapValidator(getSpaceInformation(), validity_checker)));
    }

};

} // namespace ompl

#endif
