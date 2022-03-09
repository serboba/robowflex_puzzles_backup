//
// Created by serboba on 28.01.22.
//

#include <robowflex_dart/urdf_read.h>
#include <robowflex_dart/conversion_functions.h>


void create_txt_file(std::string filename){
    Py_Initialize();
    FILE *fp;
    char *argv[2];
    argv[0] = "urdf2config.py";
    argv[1] = &filename[0];

    wchar_t ** wargv = new wchar_t *[2];

    wargv[0] = Py_DecodeLocale(argv[0], nullptr);
    wargv[1] = Py_DecodeLocale(argv[1], nullptr);

    Py_SetProgramName(wargv[0]);
    Py_Initialize();
    PySys_SetArgv(2,wargv);
    fp = fopen("urdf2config.py", "r");
    PyRun_SimpleFile(fp, "urdf2config.py");
    Py_Finalize();
    return;

}


URDF_IO::URDF_IO(std::string filename) {

    create_txt_file(filename);

    std::vector<std::string> gr_names;
    std::vector<std::vector<int>> gr_indices;
    std::vector<double> goal_p;


    std::string str = "txt_files/" + filename + ".txt";
    std::ifstream inFile(str); // CHANGE W FILENAME
    std::string gr_name, num;

    if (inFile.is_open()) {
        std::string line;
        int i = 0;
        while (std::getline(inFile, line)) {
            std::stringstream ss(line);
            if (line.compare("-") != 0) {
                std::getline(ss, gr_name, ',');
                group_names.push_back(gr_name);

                std::vector<int> gr_temp;
                int dim;
                std::getline(ss, num, ',');
                dim = std::stoi(num);
                int start;
                std::getline(ss, num, ',');
                start = std::stoi(num);

                for(int i = start; i < start+dim; i++){
                    gr_temp.push_back(i);
                }

                gr_indices.push_back(gr_temp);

            } else {
                //last line goal
                std::getline(inFile, line);
                std::stringstream ss2(line);

                for(int i = 0; i< 3; i++){
                    std::getline(ss2, num, ' ');
                    goal_pose.push_back(stod(num));
                }
                std::vector<double> rotation_;
                for(int i = 0; i<3 ; i++){
                    std::getline(ss2,num,' ');
                    rotation_.push_back(stod(num));
                }


                std::vector<double> orn = quaternion_matrix_to_stdvec(rpy_to_quaternion(rotation_[0],rotation_[1],rotation_[2]));
                // std::vector<double> orn = {0.7071055,0.7071081, 0, 0};
                //std::vector<double> orn = {1.0,0.0,0.0,0.0};
                goal_pose.insert(goal_pose.end(),orn.begin(),orn.end());
            }
        }
    }

    group_indices = gr_indices;

}

