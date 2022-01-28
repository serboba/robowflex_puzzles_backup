//
// Created by serboba on 10.01.22.
//

#ifndef ROBOWFLEX_DART_ISOMANIPULATIONOPTIMIZATION_H
#define ROBOWFLEX_DART_ISOMANIPULATIONOPTIMIZATION_H


#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/goals/GoalRegion.h>
#include <ompl/base/goals/GoalLazySamples.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace ompl
{
    namespace base
    {
        /** \brief An optimization objective which corresponds to optimizing path length. */
        class IsoManipulationOptimization : public OptimizationObjective
        {
        public:
            IsoManipulationOptimization(const SpaceInformationPtr &si, std::vector<int> group_indices);

            /** \brief Returns identity cost. */
            Cost stateCost(const State *s) const override;

            /** \brief Motion cost for this objective is defined as
                the configuration space distance between \e s1 and \e
                s2, using the method SpaceInformation::distance(). */
            Cost motionCost(const State *s1, const State *s2) const override;

            Cost motionCostHeuristic(const State *s1, const State *s2) const override;


            //Cost goalRegionCostToGo(const State *state, const Goal *goal) ;

            static  std::vector<int> changedIndex(const std::vector<double> s1, const std::vector<double> s2);

            bool isCostBetterThan(Cost c1, Cost c2) const override;
            Cost identityCost() const override;


           // Cost combineCosts(Cost c1, Cost c2) const override;
        protected:

           std::vector<int> group_indices_;
        };
    }
}

#endif //ROBOWFLEX_DART_ISOMANIPULATIONOPTIMIZATION_H

