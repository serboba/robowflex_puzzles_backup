//
// Created by serboba on 31.12.21.
//
#include <robowflex_dart/space.h>
#include <algorithm>
#include <random>
using namespace robowflex::darts;


void StateSpace::StateSampler::sampleUniform(ompl::base::State *state)
{

    auto *st = state->as<StateType>();

    int index = rng_.uniformInt(0,joints_.size()-1);

    std::vector<int> sample_group;
    for(auto const &group : groups_){
        auto it = std::find(group.begin(),group.end(),index);
        if(it != group.end()){
            sample_group = group;
            break;
        }
    }

    bool grouped_sample = false;
    if(sample_group.size() > 0)
        grouped_sample = true;

    for(int i = 0; i < joints_.size() ; i++){

        if(abs(st->values[i]) < 1e-10 || abs(st->values[i]) > 1e10 || isnan(st->values[i])) {
            st->values[i] = 0.0;
        }
        if(grouped_sample) {
            if (std::find(sample_group.begin(), sample_group.end(), i) != sample_group.end()) {
                st->values[i] = rng_.uniformReal(joints_.at(i)->getLowerLimits()(0),
                                                 joints_.at(i)->getUpperLimits()(0));
            }

        }else{ // not sampling grouped index
            if(i == index){
                st->values[i] = rng_.uniformReal(joints_.at(i)->getLowerLimits()(0),
                                                 joints_.at(i)->getUpperLimits()(0));
            }
        }

    }

/*
    auto *as = state->as<StateType>();

    for (const auto &joint : joints_)
        joint->sample(joint->getSpaceVars(as->data));
*/
 }



double StateSpace::distance_new(double v1, double v2) const{

    return (sqrt((v1-v2)*(v1-v2)));
}


int StateSpace::findIndex(std::vector<double> &distances, double t) const{
    double sum = 0.0;
    double temp_sum = 0.0;
    for(int i = 0; i < dimension_ ; i++){
        sum += distances.at(i);
        if(sum >= t ){

            return i;
        }
        // temp_sum += distances.at(i);
    }
    return (dimension_-1);
}

std::vector<double> StateSpace::getDistances(const StateSpace::StateType *const rfrom, const StateSpace::StateType *const rto)  const{
    double total_dist = 0.0;
    std::vector<double> distances_;
    for(int i = 0; i < rfrom->data.size(); i++){
        double dj = distance_new(rfrom->values[i],rto->values[i]);
        total_dist += dj;
        distances_.push_back(dj) ;
    }


    for (int j= 0; j< rfrom->data.size(); j++){
        distances_[j] /= total_dist;
    }

    return distances_;
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

    std::vector<double> distances = getDistances(rfrom,rto);

    int index = findIndex(distances,t);


    double d_interpolated = 0.0;


    for (int i = 0; i < index; i++){
        rstate->values[i] = rto->values[i];
        d_interpolated+= distances.at(i);
    }

    double s = (t-d_interpolated)/distances.at(index);


    rstate->values[index] = rfrom->values[index] + s*(rto->values[index]-rfrom->values[index]);

    for(int i = index+1; i < rfrom->data.size(); i++){
        rstate->values[i] = rfrom->values[i];
    }

    /*
    int gr_index = -1;
    for(int i = 0 ; i < grouped_indices.size(); i++) {
        if (std::find(grouped_indices.at(i).begin(), grouped_indices.at(i).end(), index) !=
            grouped_indices.at(i).end()) {
            gr_index = i;
            break;
        }
    }
    */
/*
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



