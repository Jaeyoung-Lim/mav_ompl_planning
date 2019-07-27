#ifndef OCTOMAP_OMPL_H
#define OCTOMAP_OMPL_H

#include <ompl/base/StateValidityChecker.h>

namespace ompl {

class OctomapValidityChecker : public base::StateValidityChecker {
public:
    OctomapValidityChecker(const base::SpaceInformationPtr &si) :
       base::StateValidityChecker(si)
        {
    }
    virtual bool isValid(const base::State *state) const
    {
            return true;
    }
};

} // namespace ompl

#endif
