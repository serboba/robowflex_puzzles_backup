//
// Created by serboba on 31.12.21.
//
#include <robowflex_dart/space.h>
#include <algorithm>
#include <random>
using namespace robowflex::darts;



double distance_new(double v1, double v2){

    return (sqrt((v1-v2)*(v1-v2)));
}


int findIndex(std::vector<double> distances, double t){
    double sum = 0.0;
    double temp_sum = 0.0;
    for(size_t i = 0; i < distances.size() ; i++){
        sum += distances.at(i);
        if(sum >= t ){
            return i;
        }
    }
    return distances.size()-1;
}


std::vector<double> getDistances(std::vector<double> rfrom, std::vector<double> rto)  {
    double total_dist = 0.0;
    std::vector<double> distances;
    for(size_t i = 0; i < rfrom.size(); i++){
        double dj = distance_new(rfrom.at(i),rto.at(i));
        distances.push_back(dj);
        total_dist += dj;
    }

    for (int j= 0; j< rfrom.size(); j++){
        distances[j] /= total_dist;
        //   std::cout << "normalized : " << distances[j] << std::endl;
    }
    return distances;
}

std::vector<std::vector<int>> StateSpace::findGroupIndex() const {

    std::vector<std::string> group_names = getGroups();
    std::vector<std::vector<int>> group_indices;
    ompl::base::State *temp_state = allocState();
    std::vector<double> temp_values;

    for(size_t i = 0; i < getDimension(); i++)
        temp_values.push_back(0.0);

    copyFromReals(temp_state,temp_values);

    for (size_t i = 0; i < group_names.size(); i++) {
        int groupDim = getGroupDimension(group_names[i]);

        if (groupDim > 1) {
            //FOUND GROUP

            Eigen::VectorXd temp_group(groupDim);
            temp_group << -100.0,-100.0;  // random ???
            setGroupState(group_names[i],temp_state,temp_group);
            copyToReals(temp_values, temp_state);
            std::vector<int> group_index;
            int index_it = 0;
            for (size_t j = 0; j < temp_values.size(); j++) {
                if (temp_values.at(j) == temp_group(index_it)) {
                    group_index.push_back(j);
                    index_it++;
                }
                if(index_it ==groupDim)
                    break;
            }
            group_indices.push_back(group_index);
        }
    }
    freeState(temp_state);
    return group_indices;
}


std::vector<int> StateSpace::shuf(int dim) const {

    std::vector<int> shuffleIndices;
    for(size_t i =0 ; i < dim ; i++)
        shuffleIndices.push_back(i);

    std::random_shuffle(shuffleIndices.begin(),shuffleIndices.end());

    return shuffleIndices;
}

void shuffleState(std::vector<double> &st, std::vector<int> shuffleIndex){
    std::vector<double> shuffled_st;
    for(size_t i = 0; i < shuffleIndex.size() ; i++){
        shuffled_st.push_back(st.at(shuffleIndex.at(i)));
    }
    st = shuffled_st;
}

void reverseShuffle(std::vector<double> &st, std::vector<int> shuffleIndex){
    std::vector<double> reverse;
    for(size_t i = 0; i< shuffleIndex.size(); i++){
        reverse.push_back(0.0);
    }

    for(size_t i = 0; i<shuffleIndex.size(); i++){
        reverse[shuffleIndex.at(i)] =  st.at(i);
    }
    st= reverse;
}



void StateSpace::interpolate(const ompl::base::State *from, const ompl::base::State *to, double t,  // WITH SHUFFLE
                             ompl::base::State *state) const
{

    const auto &rfrom = from->as<StateSpace::StateType>();
    const auto &rto = to->as<StateSpace::StateType>();
    auto *rstate = state->as<StateSpace::StateType>();

    std::vector<double> rfrom_,rstate_,rto_;
    copyToReals(rstate_,rfrom);
    copyToReals(rfrom_,rfrom);
    copyToReals(rto_,rto);


    std::vector<int> shuffleIndices = shuf(rfrom_.size());

    shuffleState(rfrom_,shuffleIndices);
    shuffleState(rto_,shuffleIndices);

    std::vector<std::vector<int>> groupIndex = findGroupIndex();
    std::vector<int> group_index = groupIndex.at(0);


    std::vector<double> distances = getDistances(rfrom_, rto_);
    int index = findIndex(distances,t);

    double d_interpolated = 0.0;

    for (int i = 0; i < index; i++){
        rstate_[i] = rto_[i];
        d_interpolated+= distances.at(i);
    }

    double s = (t-d_interpolated)/distances.at(index);

//    std::cout << " d_interpolated : " << d_interpolated << " , t : " << t << ", s: " << s << ", index: "<< index
//             << ", s/dist:" << s/distances.at(index) << ", rfrom+s/dist : " <<rfrom->values[index] + s*(rto->values[index]-rfrom->values[index]) <<   std::endl;

    rstate_[index] = rfrom_[index] + s*(rto_[index]-rfrom_[index]);

    for(size_t i = index+1; i < rfrom_.size(); i++){
        rstate_[i] = rfrom_[i];
    }


    reverseShuffle(rstate_,shuffleIndices);
    reverseShuffle(rfrom_,shuffleIndices);
    reverseShuffle(rto_,shuffleIndices);

    copyFromReals(rstate,rstate_);

/*
    std::cout << "from: ";
    for(int i = 0; i < rfrom_.size(); i++)
        std::cout << rfrom_.at(i) << ", ";
    std::cout << std::endl;


    std::cout << "to: ";
    for(int i = 0; i < rto_.size(); i++)
        std::cout << rto_.at(i) << ", ";
    std::cout << std::endl;

    std::cout << "out: ";
    for(int i = 0; i < rstate_.size(); i++)
        std::cout << rstate_.at(i) << ", ";
    std::cout <<"\n" << std::endl;
*/
}



/*
void StateSpace::interpolate(const ompl::base::State *from, const ompl::base::State *to, double t,  // WITHOUT SHUFFLE
                             ompl::base::State *state) const {
    const auto &rfrom = from->as<StateSpace::StateType>();
    const auto &rto = to->as<StateSpace::StateType>();
    auto *rstate = state->as<StateSpace::StateType>();

    std::vector<double> rfrom_,rstate_,rto_;
    copyToReals(rstate_,rfrom);
    copyToReals(rfrom_,rfrom);
    copyToReals(rto_,rto);

    std::vector<std::vector<int>> groupIndex = findGroupIndex();
    std::vector<int> group_index = groupIndex.at(0);


    std::vector<double> distances = getDistances(rfrom_, rto_);
    int index = findIndex(distances,t);

    double d_interpolated = 0.0;

    for (int i = 0; i < index; i++){
        rstate_[i] = rto_[i];
        d_interpolated+= distances.at(i);
    }

    double s = (t-d_interpolated)/distances.at(index);

//    std::cout << " d_interpolated : " << d_interpolated << " , t : " << t << ", s: " << s << ", index: "<< index
//             << ", s/dist:" << s/distances.at(index) << ", rfrom+s/dist : " <<rfrom->values[index] + s*(rto->values[index]-rfrom->values[index]) <<   std::endl;

    rstate_[index] = rfrom_[index] + s*(rto_[index]-rfrom_[index]);

    for(size_t i = index+1; i < rfrom_.size(); i++){
        rstate_[i] = rfrom_[i];
    }

    copyFromReals(rstate,rstate_);

}

 */
