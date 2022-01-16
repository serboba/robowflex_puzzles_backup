//
// Created by serboba on 17.12.21.
//

#ifndef ROBOWFLEX_DART_ISOMANIPULATIONSTATESPACE_H
#define ROBOWFLEX_DART_ISOMANIPULATIONSTATESPACE_H


#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <Eigen/Dense>
#include "ompl/base/StateSpace.h"

namespace ompl{
    namespace base{
        class IsoManipulationStateSpace : public ompl::base::RealVectorStateSpace{

        public:

            IsoManipulationStateSpace(int i) : ompl::base::RealVectorStateSpace(i){

            }

            double distance_new(double v1, double v2) const{

                return (sqrt((v1-v2)*(v1-v2)));
            }


            int findIndex(std::vector<double> distances, double t) const{
                double sum = 0.0;
                double temp_sum = 0.0;
                for(int i = 0; i < distances.size() ; i++){ // 0.4 , 0.02,0, 0.5
                    sum += distances.at(i);
                    if(sum >= t ){
                        return i;
                    }
                }
                return 0;
            }

            std::vector<double> getDistances( const ompl::base::State *from,
                                              const ompl::base::State  *to) const {
                double total_dist = 0.0;
                std::vector<double> distances;

                const auto *rfrom = from->as<StateType>();
                const auto *rto = to->as<StateType>();

                for(int i = 0; i < 4; i++){
                    double dj = distance_new(rfrom->values[i],rto->values[i]);
                    distances.push_back(dj);
                    total_dist += dj;
                }


                for (int j= 0; j< 4; j++){
                    distances[j] /= total_dist;
                    //   std::cout << "normalized : " << distances[j] << std::endl;
                }
                return distances;
            }

            void interpolate(const ompl::base::State *from, const ompl::base::State *to, double t,
                             ompl::base::State *state) const override
            {
                
                const auto &rfrom = from->as<StateType>();
                const auto &rto = to->as<StateType>();
                auto *rstate = state->as<StateType>();


                std::vector<double> distances = getDistances(from, to);
                int index = findIndex(distances,t);

                double d_interpolated = 0.0;

                for (int i = 0; i < index; i++){
                    rstate->values[i] = rto->values[i];
                    d_interpolated+= distances.at(i);
                }

                double s = (t-d_interpolated)/distances.at(index);

                std::cout << " d_interpolated : " << d_interpolated << " , t : " << t << ", s: " << s << ", index: "<< index
                          << ", s/dist:" << s/distances.at(index) << ", rfrom+s/dist : " <<rfrom->values[index] + s*(rto->values[index]-rfrom->values[index]) <<   std::endl;

                rstate->values[index] = rfrom->values[index] + s*(rto->values[index]-rfrom->values[index]);

                for(int i = index+1; i < 4; i++){
                    rstate->values[i] = rfrom->values[i];
                }


            }


        };

    }
}

#endif //ROBOWFLEX_DART_ISOMANIPULATIONSTATESPACE_H
