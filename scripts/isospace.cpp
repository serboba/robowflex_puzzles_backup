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
    for(int i = 0; i < distances.size() ; i++){
        sum += distances.at(i);
        if(sum >= t){

            return i;
        }
    }
    return 0;
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



    std::vector<double> distances = getDistances(rfrom, rto);
    int index = findIndex(distances,t);

    double d_interpolated = 0.0;

    for (int i = 0; i < index; i++){

        rstate->values[i] = rto->values[i];
        d_interpolated+= distances.at(i);
    }

    double s = t-d_interpolated;
    std::cout << " d_interpolated : " << d_interpolated << " , t : " << t << ", s: " << s << ", index: "<< index
              << ", s/dist:" << s/distances.at(index) << ", rfrom+s/dist : " <<rfrom->values[index] + s/(rto->values[index]-rfrom->values[index]) <<   std::endl;

    //rstate->values[index] = rfrom->values[index] + s/distances.at(index);

    rstate->values[index] = rfrom->values[index] + s*(rto->values[index]-rfrom->values[index]);

    for(int i = index+1; i < rfrom->data.size(); i++){
        rstate->values[i] = rfrom->values[i];
    }

    std::cout << "from: ";
    for(int i = 0; i < 4; i++)
        std::cout << rfrom->values[i] << ", ";
    std::cout << std::endl;


    std::cout << "to: ";
    for(int i = 0; i < 4; i++)
        std::cout << rto->values[i] << ", ";
    std::cout << std::endl;

    std::cout << "out: ";
    for(int i = 0; i < 4; i++)
        std::cout << rstate->values[i] << ", ";
    std::cout <<"\n" << std::endl;

}
