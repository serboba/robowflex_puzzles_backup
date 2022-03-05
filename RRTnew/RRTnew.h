//
// Created by serboba on 22.01.22.
//

#ifndef ROBOWFLEX_DART_RRTNEW_H
#define ROBOWFLEX_DART_RRTNEW_H

#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/geometric/planners/PlannerIncludes.h>
#include <robowflex_dart/IsoManipulationOptimization.h>

#include <robowflex_dart/space.h>
#include <fstream>
namespace ompl
{
    namespace geometric
    {
        /**
           @anchor gRRTC
           @par Short description
           The basic idea is to grow two RRTs, one from the start and
           one from the goal, and attempt to connect them.
           @par External documentation
           J. Kuffner and S.M. LaValle, RRT-connect: An efficient approach to single-query path planning, in <em>Proc.
           2000 IEEE Intl. Conf. on Robotics and Automation</em>, pp. 995–1001, Apr. 2000. DOI:
           [10.1109/ROBOT.2000.844730](http://dx.doi.org/10.1109/ROBOT.2000.844730)<br>
           [[PDF]](http://ieeexplore.ieee.org/ielx5/6794/18246/00844730.pdf?tp=&arnumber=844730&isnumber=18246)
           [[more]](http://msl.cs.uiuc.edu/~lavalle/rrtpubs.html)
        */

        /** \brief RRT-Connect (RRTConnect) */
        class RRTnew : public base::Planner
        {
        public:
            /** \brief Constructor */
            RRTnew(const base::SpaceInformationPtr &si, std::vector<std::vector<int>> group_indices ,bool addIntermediateStates = false,
                   bool useIsolation = false);

            ~RRTnew() override;

            void getPlannerData(base::PlannerData &data) const override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            void clear() override;

            /** \brief Return true if the intermediate states generated along motions are to be added to the tree itself
             */
            bool getIntermediateStates() const
            {
                return addIntermediateStates_;
            }

            ompl::base::Cost bestCost() const
            {
                return bestCost_;
            }
            /** \brief Specify whether the intermediate states generated along motions are to be added to the tree
             * itself */
            void setIntermediateStates(bool addIntermediateStates)
            {
                addIntermediateStates_ = addIntermediateStates;
            }

            /** \brief Set the range the planner is supposed to use.

                This parameter greatly influences the runtime of the
                algorithm. It represents the maximum length of a
                motion to be added in the tree of motions. */
            void setRange(double distance)
            {
                maxDistance_ = distance;
            }

            /** \brief Get the range the planner is using */
            double getRange() const
            {
                return maxDistance_;
            }

            /** \brief Set a different nearest neighbors datastructure */
            template <template <typename T> class NN>
            void setNearestNeighbors()
            {
                if ((tStart_ && tStart_->size() != 0) || (tGoal_ && tGoal_->size() != 0))
                    OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                clear();
                tStart_ = std::make_shared<NN<Motion *>>();
                tGoal_ = std::make_shared<NN<Motion *>>();
                setup();
            }


            unsigned int numIterations() const
            {
                return iterations_;
            }
/*
            ompl::base::Cost bestCost() const
            {
                return bestCost_;
            }
*/
            void setup() override;

        protected:
            /** \brief Representation of a motion */
            class Motion
            {
            public:
                Motion() = default;

                Motion(const base::SpaceInformationPtr &si) : state(si->allocState())
                {
                }

                ~Motion() = default;

                const base::State *root{nullptr};
                base::State *state{nullptr};
                Motion *parent{nullptr};
                base::Cost cost;
                int index_changed;
            };

            /** \brief A nearest-neighbor datastructure representing a tree of motions */
            using TreeData = std::shared_ptr<NearestNeighbors<Motion *>>;

            /** \brief Information attached to growing a tree of motions (used internally) */
            struct TreeGrowingInfo
            {
                base::State *xstate;
                Motion *xmotion;
                bool start;
            };

            /** \brief The state of the tree after an attempt to extend it */
            enum GrowState
            {
                /// no progress has been made
                TRAPPED,
                /// progress has been made towards the randomly sampled state
                ADVANCED,
                /// the randomly sampled state was reached
                REACHED
            };

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state, b->state);
            }

            /** \brief Grow a tree towards a random state */
            GrowState growTree(TreeData &tree, TreeGrowingInfo &tgi, Motion *rmotion);


            int rewireTree(Motion *startMotion, Motion *goalMotion);



            void getMotionVectors(Motion * mot_,std::vector<Motion*> &vec);

            void
            getIntermediateState(const base::State *from, const base::State *to, base::State *state, int index_group);

            /** \brief State sampler */
            base::StateSamplerPtr sampler_;

            /** \brief The start tree */
            TreeData tStart_;

            /** \brief The goal tree */
            TreeData tGoal_;

            /** \brief A flag that toggles between expanding the start tree (true) or goal tree (false). */
            bool startTree_{true};

            /** \brief The maximum length of a motion to be added to a tree */
            double maxDistance_{0.};

            /** \brief Flag indicating whether intermediate states are added to the built tree of motions */
            bool addIntermediateStates_;
            bool useIsolation_;
            /** \brief The random number generator */
            RNG rng_;

            /** \brief The pair of states in each tree connected during planning.  Used for PlannerData computation */
            std::pair<base::State *, base::State *> connectionPoint_;

            /** \brief Distance between the nearest pair of start tree and goal tree nodes. */
            double distanceBetweenTrees_;

            base::OptimizationObjectivePtr opt_;


            base::Cost bestCost_{std::numeric_limits<double>::quiet_NaN()};
            base::Cost incCost{0};

            std::vector<ompl::base::State*> bestPath;
            std::string bestCostProgressProperty() const;


            std::vector<std::vector<int>> group_indices;
            void createNewMotion(const base::State *st, Motion *premotion,
                                    ompl::geometric::RRTnew::Motion *newmotion);

            bool validMotionCheck(const bool start, const base::State *from_, const base::State *to_);


            unsigned int iterations_{0u};


            int getChangedIndex(const base::State *from, const base::State *to);

            std::vector<base::State *> getStates(std::vector<Motion *>);

            //  std::vector<base::State *> rewire(std::vector<base::State *> mainStates);



            void reConnect(ompl::base::State *from,
                                                        std::vector<std::pair<ompl::base::State *,int >> &prio_,
                                                        std::vector<std::pair<ompl::base::State *,int >> &stack_,
                                                        std::vector<ompl::base::State *> &rewireResult);


            std::vector<int>
            getChangedGroups(const std::vector<double> &from_, const std::vector<double> &to_);

            void getChangedIndices(const base::State *rfrom, const base::State *rto, std::vector<int> &indices_) const;

            int rewire(std::vector<base::State *> &mainPath);

            void
            reConnect( base::State *from, std::vector<std::pair<ompl::base::State *, int>> &queue_,
                       std::vector<ompl::base::State *> &rewireResult);


            int getCostPath(std::vector<base::State *> &states_);

            std::string numIterationsProperty() const
            {
                return std::to_string(numIterations());
            }


            std::vector<int> getChangedGroups(const base::State *rfrom, const base::State *rto);

            void simplifyActionIntervals(std::vector<ompl::base::State *> &mainPath);

            void isolateStates(const base::State *rfrom, const base::State *rto,std::vector<ompl::base::State *> &iso_);

            void buildIsoStates(const std::vector<double> &from_, const std::vector<double> &to_,
                                                      std::vector<int> &changed_index_groups,std::vector<ompl::base::State *> &iso_);

            void simplifyPath(std::vector<ompl::base::State *> &path);

            void checkRepairPath(std::vector<ompl::base::State *> &path_);

        };
    }
}

#endif //ROBOWFLEX_DART_RRTNEW_H

