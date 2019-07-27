#ifndef OCTOMAP_OMPL_H
#define OCTOMAP_OMPL_H

#include <ompl/base/StateValidityChecker.h>

namespace ompl {

class OctomapValidityChecker : public base::StateValidityChecker {
public:
    OctomapValidityChecker(const base::SpaceInformationPtr &space_info) :
       base::StateValidityChecker(space_info)
        {
    }
    virtual bool isValid(const base::State *state) const
    {
            return true;
    }
};

class OctomapValidator : public base::MotionValidator {
 public:
  OctomapValidator(const base::SpaceInformationPtr& space_info,
      typename std::shared_ptr<OctomapValidityChecker> validity_checker)
      : base::MotionValidator(space_info), validity_checker_(validity_checker) {
  }

  virtual bool checkMotion(const base::State* s1, const base::State* s2) const {
    std::pair<base::State*, double> unused;
    return checkMotion(s1, s2, unused);
  }

  virtual bool checkMotion(const base::State* s1, const base::State* s2,
                           std::pair<base::State*, double>& last_valid) const {
      return true;
    }

 protected:
  typename std::shared_ptr<OctomapValidityChecker> validity_checker_;
};

} // namespace ompl

#endif
