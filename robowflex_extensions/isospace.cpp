//
// Created by serboba on 31.12.21.
//
#include <robowflex_dart/space.h>
#include <algorithm>
#include <random>
using namespace robowflex::darts;



void StateSpace::StateSampler::sampleUniform(ompl::base::State *state)
{
    auto *st = state->as<StateSpace::StateType>();
    int index = rng_.uniformInt(0, groups_.size()-1);

    for(int i = 0; i < st->data.size() ; i++){
        if(abs(st->values[i]) < 1e-10 || abs(st->values[i]) > 1e10 || isnan(st->values[i])) {
            st->values[i] = 0.0;
        }
    }

    for(auto &i : groups_.at(index)){
        st->values[i] = rng_.uniformReal(joints_.at(i)->getLowerLimits()(0),
                                         joints_.at(i)->getUpperLimits()(0));
    }
/*
    std::cout <<"sample: " << std::endl;
    space_->printState(state);
*/
 }



double StateSpace::distance(double v1, double v2) const{

   //double d =  sqrt((v1-v2)*(v1-v2)); // EUCLIDEAN
    double d =  abs(v1-v2);             // MANHATTAN
    return (d<1e-8) ? 0.0 : d;
}


int StateSpace::findIndex(std::vector<double> &distances, double t) const{
    double sum = 0.0;
    for(size_t i = 0; i < grouped_indices.size() ; i++){
        sum += distances.at(i);
        if(sum >= t ){
            return i;
        }
        // temp_sum += distances.at(i);
    }
    return (grouped_indices.size()-1);
}



std::vector<double> StateSpace::getDistances(const StateSpace::StateType *const rfrom, const StateSpace::StateType *const rto)  const{
    std::vector<double> distances_;
    double total_dist = 0.0;
    for(auto const &group : grouped_indices){
        double dist_group = 0.0;
        for(auto const &index : group){
            double dj = distance(rfrom->values[index],rto->values[index]);
            total_dist += dj;
            dist_group += dj;
        }
        distances_.push_back(dist_group);
    }

   for(size_t j = 0; j<distances_.size(); j++){
       if(distances_[j] > 1e-8)
           distances_[j] /= total_dist;
       else
           distances_[j] = 0.0;
   }
   return distances_;

}


double StateSpace::distance(const ompl::base::State *state1, const ompl::base::State *state2) const
{
    const auto &s1 = state1->as<StateType>();
    const auto &s2 = state2->as<StateType>();

    double d = 0;

    for(int i = 0; i< s1->data.size(); i++){
        d+= distance (s1->values[i],s2->values[i]);
    }

    return d;
}



void StateSpace::interpolate(const ompl::base::State *from, const ompl::base::State *to, double t,
                                       ompl::base::State *state) const
{

    const auto &rfrom = from->as<StateSpace::StateType>();
    const auto &rto = to->as<StateSpace::StateType>();
    auto *rstate = state->as<StateSpace::StateType>();
/*
    std::cout << "fromB: ";
    for(int i = 0; i < dimension_; i++)
        std::cout << rfrom->values[i] << ", ";
    std::cout << std::endl;
    std::cout << "toB: ";
    for(int i = 0; i < dimension_; i++)
        std::cout << rto->values[i] << ", ";
    std::cout << std::endl;
*/

    std::vector<double> distances = getDistances(rfrom,rto);  // vorher 0.5, 0.3, 0.2, 0.4 jetzt 0.5, 0.5, 0.4 findindex 1

    int index = findIndex(distances,t);
    double d_interpolated = 0.0;


 //   std::cout << "index INT: " << index<< std::endl;
    for (int i = 0; i < index; i++){
        for(auto const &index_in_group : grouped_indices.at(i)){
            rstate->values[index_in_group] = rto->values[index_in_group];
        }
        d_interpolated+= distances.at(i);
    }

    double s = (t-d_interpolated)/distances.at(index);
    for(auto const &index_in_group : grouped_indices.at(index)){
        rstate->values[index_in_group] = rfrom->values[index_in_group] + s*(rto->values[index_in_group]-rfrom->values[index_in_group]);
    }



    for(size_t i = index+1; i < grouped_indices.size(); i++){

        for(auto const &index_in_group : grouped_indices.at(i)){
            rstate->values[index_in_group] = rfrom->values[index_in_group];
        }
    }
/*
    std::cout << "t : " << t << std::endl;

    std::cout << "from: ";
    for(int i = 0; i < dimension_; i++)
        std::cout << rfrom->data(i) << ", ";
    std::cout << std::endl;
    std::cout << "to: ";
    for(int i = 0; i < dimension_; i++){
        std::cout << rto->data(i) << ", ";
    }
    std::cout << std::endl;
    std::cout << "out: ";
    for(int i = 0; i < dimension_; i++)
        std::cout << rstate->data(i) << ", ";
    std::cout <<"\n" << std::endl;
*/

}






