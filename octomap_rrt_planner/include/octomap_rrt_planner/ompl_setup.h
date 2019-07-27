#ifndef OMPL_SETUP_H
#define OMPL_SETUP_H

#include <ompl/geometric/SimpleSetup.h>
#include "ompl/base/SpaceInformation.h"
#include <ompl/base/spaces/SE3StateSpace.h>

namespace ompl {

class OmplSetup : public geometric::SimpleSetup {
    public:
        OmplSetup() : geometric::SimpleSetup(base::StateSpacePtr(new base::RealVectorStateSpace(3))) {}

};

} // namespace ompl

#endif
