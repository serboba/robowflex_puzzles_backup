//
// Created by serboba on 31.12.21.
//
#include <robowflex_dart/isospace.h>

using namespace robowflex::darts;

double distance_new(double v1, double v2){

    return (sqrt((v1-v2)*(v1-v2)));
}


int findIndex(std::vector<double> distances, double t){
    double sum = 0.0;
    double temp_sum = 0.0;
    for(int i = 0; i < distances.size() ; i++){ // 0.4 , 0.02,0, 0.5
        sum += distances.at(i);
        //  std::cout << " test :" << std::endl;
        //  if(sum >= t && temp_sum>= 0.0 && temp_sum < 0.01){
        if(sum >= t ){

            return i;
        }
        // temp_sum += distances.at(i);
    }
    return distances.size()-1;
}

std::vector<double> getDistances(const StateSpace::StateType *const rfrom, const StateSpace::StateType *const rto)  {
    double total_dist = 0.0;
    std::vector<double> distances;
    for(int i = 0; i < rfrom->data.size(); i++){
        double dj = distance_new(rfrom->values[i],rto->values[i]);
        distances.push_back(dj);
        total_dist += dj;
    }


    for (int j= 0; j< rfrom->data.size(); j++){
        distances[j] /= total_dist;
        //   std::cout << "normalized : " << distances[j] << std::endl;
    }
    return distances;
}


void robowflex::darts::interpolate_iso(const ompl::base::State *from, const ompl::base::State *to, double t,
                             ompl::base::State *state)
{
    const auto &rfrom = from->as<StateSpace::StateType>();
    const auto &rto = to->as<StateSpace::StateType>();
    auto *rstate = state->as<StateSpace::StateType>();
   // std::vector<std::pair<int,int>> groupIndex = findGroupIndex(rfrom, rto);

    std::vector<double> distances = getDistances(rfrom, rto);

    int index = findIndex(distances,t);

    double d_interpolated = 0.0;

    for (int i = 0; i < index; i++){
        rstate->values[i] = rto->values[i];
        d_interpolated+= distances.at(i);
    }

    double s = (t-d_interpolated)/distances.at(index);

//    std::cout << " d_interpolated : " << d_interpolated << " , t : " << t << ", s: " << s << ", index: "<< index
//             << ", s/dist:" << s/distances.at(index) << ", rfrom+s/dist : " <<rfrom->values[index] + s*(rto->values[index]-rfrom->values[index]) <<   std::endl;

    rstate->values[index] = rfrom->values[index] + s*(rto->values[index]-rfrom->values[index]);

    for(int i = index+1; i < rfrom->data.size(); i++){
        rstate->values[i] = rfrom->values[i];
    }

/*
    std::cout << "from: ";
    for(int i = 0; i < 5; i++)
        std::cout << rfrom->values[i] << ", ";
    std::cout << std::endl;


    std::cout << "to: ";
    for(int i = 0; i < 5; i++)
        std::cout << rto->values[i] << ", ";
    std::cout << std::endl;

    std::cout << "out: ";
    for(int i = 0; i < 5; i++)
        std::cout << rstate->values[i] << ", ";
    std::cout <<"\n" << std::endl;
*/
}




/*
std::vector<std::pair<int,int>> StateSpace::findGroupIndex(const StateSpace::StateType *const rfrom,
                                                           const StateSpace::StateType *const rto) const {
    std::vector<std::string> group_names = getGroups();
    std::vector<std::pair<int,int>> indexSize;

    // std::cout << " start" << std::endl;
    for(int i = 0; i < group_names.size(); i++){
        //   std::cout << group_names[i] << std::endl;

        Eigen::VectorXd sub_group(getGroupDimension(group_names[i]));
        getGroupState(group_names[i],rfrom, sub_group);

        int size_n = 0;
        int start_index = -1;

        for (auto const& joint : group_joints_.find(group_names[i])->second) {
            if(start_index == -1)
                start_index = joint->getStartInSpace();
            size_n++;
        }
        //  std::cout << " group size: " << size_n << std::endl;
        //  std::cout << " start index: " << start_index << std::endl;
        indexSize.push_back(std::make_pair(start_index,size_n));
    }

    return indexSize;
}
*/


/* TRASH

 int * shuffleIndex(int s_size, int shuffle_index[]) {

    for(int i =0; i < s_size; i++)
        shuffle_index[i] = i;

    std::random_shuffle(&shuffle_index[0],&shuffle_index[s_size-1]);

    return shuffle_index;
}

void shuffleStates(const StateSpace::StateType *const rfrom, const StateSpace::StateType *const rto, int *indexarr) {
    Eigen::VectorXd temp_from(rfrom->data.size());
    Eigen::VectorXd temp_to(rfrom->data.size());

    for(int i = 0; i < rfrom->data.size() ; i++){
        temp_from(i) = rfrom->values[i];
        temp_to(i) = rto->values[i];
    }

    for(int i = 0; i < rfrom->data.size() ; i++){
        rfrom->values[i] = temp_from(indexarr[i]);
        rto->values[i] = temp_to(indexarr[i]);
    }

}

void undoShuffle(const StateSpace::StateType *const rout, int *indexarr){
    Eigen::VectorXd temp_out(rout->data.size());


    for(int i = 0; i < rout->data.size() ; i++){
        temp_out(i) = rout->values[i];
    }

    for(int i = 0; i < rout->data.size() ; i++){
        rout->values[i] = temp_out(indexarr[i]);
    }

}






    std::cout << "fromBEG: ";
    for(int i = 0; i < 5; i++)
        std::cout << rfrom->values[i] << ", ";
    std::cout << std::endl;


    std::cout << "toBEG: ";
    for(int i = 0; i < 5; i++)
        std::cout << rto->values[i] << ", ";
    std::cout << std::endl;


    Eigen::VectorXd temp_from(rfrom->data.size());
    Eigen::VectorXd temp_to(rfrom->data.size());

    for(int i = 0; i < rfrom->data.size() ; i++){
        temp_from(i) = rfrom->values[i];
        temp_to(i) = rto->values[i];
    }



    int p[rfrom->data.size()];
    int *random_index = shuffleIndex(rfrom->data.size(), p);

    //Eigen::VectorXd temp(rfrom->data.size());


    shuffleStates(rfrom, rto, random_index);

 undoShuffle(rstate,random_index);

    for(int i = 0; i < rstate->data.size() ; i++){
        rfrom->values[i] = temp_from(i);
        rto->values[i] = temp_to(i);
    }


 */



/*
 int sizeflag = 0;
    int it_;

    for(auto const &temp_ : groupIndex){
        if(index== temp_.first)
            if(temp_.second >1){
                sizeflag = 1;
                std::cout << "im in for index : " << index << " - temp sec : " << temp_.second << std::endl;
                it_ = temp_.second;
                break;
            }

    }

    if(sizeflag == 0){
        double s = (t-d_interpolated)/distances.at(index);

        std::cout << " d_interpolated : " << d_interpolated << " , t : " << t << ", s: " << s << ", index: "<< index
                  << ", s/dist:" << s/distances.at(index) << ", rfrom+s/dist : " <<rfrom->values[index] + s*(rto->values[index]-rfrom->values[index]) <<   std::endl;


        rstate->values[index] = rfrom->values[index] + s*(rto->values[index]-rfrom->values[index]);

        std::cout << "no found" << std::endl;
    }
    else{
       // const int index_ =groupIndex[index].second;
        for(int i = 0; i < it_ ; i++){

            double s = (t-d_interpolated)/distances.at(index);

            if(t-d_interpolated > 0){
                std::cout << " d_interpolated : " << d_interpolated << " , t : " << t << ", s: " << s << ", index: "<< index
                          << ", s/dist:" << s/distances.at(index) << ", rfrom+s/dist : " <<rfrom->values[index] + s*(rto->values[index]-rfrom->values[index]) <<   std::endl;

                rstate->values[index] = rfrom->values[index] + s*(rto->values[index]-rfrom->values[index]);

                std::cout << "running time: " << i << std::endl;
                std::cout << "groupIndex[in]: " << it_ << std::endl;

                d_interpolated += distances.at(index);

                std::cout << "d_interpol: " << d_interpolated << " - t : " << t << std::endl;
                index++;
            }

        }
    }



 */

