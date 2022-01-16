//
// Created by serboba on 10.01.22.
//

#ifndef ROBOWFLEX_DART_ISOMANIPULATIONOPTIMIZATION_H
#define ROBOWFLEX_DART_ISOMANIPULATIONOPTIMIZATION_H


#include <ompl/base/OptimizationObjective.h>

namespace ompl
{
    namespace base
    {
        /** \brief An optimization objective which corresponds to optimizing path length. */
        class IsoManipulationOptimization : public OptimizationObjective
        {
        public:
            IsoManipulationOptimization(const SpaceInformationPtr &si);

            /** \brief Returns identity cost. */
            Cost stateCost(const State *s) const override;

            /** \brief Motion cost for this objective is defined as
                the configuration space distance between \e s1 and \e
                s2, using the method SpaceInformation::distance(). */
            Cost motionCost(const State *s1, const State *s2) const override;

            Cost motionCostHeuristic(const State *s1, const State *s2) const override;

        };
    }
}

#endif //ROBOWFLEX_DART_ISOMANIPULATIONOPTIMIZATION_H
