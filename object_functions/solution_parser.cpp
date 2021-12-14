//
// Created by serboba on 12.12.21.
//


#include <robowflex_dart/solution_analyzer.h>

std::vector<std::vector<double>> read_solution_to_matrix(){


    std::ifstream inFile("solution_path_table.txt"); // CHANGE W FILENAME
    std::string line;
    std::stringstream ss(line);
    std::vector<std::vector<double>> vec;

    while(std::getline(inFile,line)){
        std::vector<double> line_data;
        std::stringstream lineStream(line);
        double value;
        while(lineStream >> value)
            line_data.push_back(value);
        vec.push_back(line_data);
    }

    return vec;

}

std::vector<MatrixXd> divide_vector_to_matrix(std::vector<std::vector<double>> vec) {
    int rows = vec.size(); // columns
    int cols = vec[0].size();
    std::cout << "rows - " << rows << " - cols  - " << cols << std::endl;

    std::vector<MatrixXd> values_separated;
    for (int i = 0; i < cols ; i++) {
        MatrixXd path(rows,1);
        for(int j = 0 ; j < rows ; j++) {
            path(j,0) = vec[j][i];
        }
        std::cout << "path : " << path << std::endl;
        values_separated.push_back(path);
    }
    return values_separated;
}

std::vector<double> find_minima_maxima(MatrixXd path){
    // find min max in array
    std::vector<double> min_max; // todo maybe seperate minima maxima?
    std::vector<double> mini;
    std::vector<double> maxi;
    for(int i = 1; i < path.rows()-1; i++){
        if((path(i-1) > path(i)) && (path(i) < path(i+1)))
            mini.push_back(path(i));
        else if ((path(i-1) < path(i)) && (path(i) > path(i+1)))
            maxi.push_back(path(i));
    }

    //last point always should be checkedneeded
    if(path(path.rows()-1) > path(path.rows()-2))
        maxi.push_back(path(path.rows()-1));
    else if (path(path.rows()-1) < path(path.rows()-2))
        mini.push_back(path(path.rows()-1));

    /*
    for(double k : mini){
        std::cout<< "Found minimum: " << k << std::endl;
    }


    for(double k : maxi){
        std::cout<< "Found maximum: " << k << std::endl;
    }
    std::cout << "\n\n NEXT JOINT \n\n" <<std::endl;
    */
    mini.insert(mini.end(),maxi.begin(),maxi.end());

    for(double k : mini){
        std::cout<< "Found value: " << k << std::endl;
    }
    std::cout << "\n\n NEXT JOINT \n\n" <<std::endl;
    return mini;
}


std::vector<std::pair<std::string,std::vector<double>>> get_domain_values(std::vector<std::string> joint_names){
    std::vector<MatrixXd> paths = divide_vector_to_matrix(read_solution_to_matrix());

    std::vector<std::pair<std::string,std::vector<double>>> values;

    for (int i = 0; i < joint_names.size(); i++) { // todo find a better way
        if(joint_names[i] == "cube_gr1" || joint_names[i] == "cube_gr2" )
            continue;
        values.push_back(std::make_pair(joint_names[i],find_minima_maxima(paths[i+2]))); // cubes are the first two joint i dont know a better way to pass them
    }
    return values;
}

void main_test(){
    auto div_mat = divide_vector_to_matrix(read_solution_to_matrix());

}