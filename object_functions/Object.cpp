//
// Created by serboba on 12.11.21.
//

#include <robowflex_dart/Object.h>


boost::filesystem::path pp(boost::filesystem::current_path().parent_path().parent_path().parent_path());
const std::string abs_path = pp.string() + "/src/robowflex/robowflex_dart/include/io/";


void create_txt_from_urdf(){
    // RUN URDF2TXT

    std::string script = "python " + abs_path + "urdf2txt.py ";
    std::system(script.c_str());
    //std::system("python urdf2txt.py");

}

std::vector<double> read_into_vector(std::string str){
    std::istringstream ss(str);
    std::string s;
    std::vector<double> vec;
    int i = 0;
    while(std::getline(ss,s,' ') && i < 3){
        double temp = std::stod(s);
        vec.push_back(temp);
        i++;
    }
    return vec;
}


void read_obj_txt_file(std::string filename,std::vector<Object> &objects_){
    std::string str = abs_path+"obj_txt/" + filename+".txt";
    std::ifstream inFile(str); // CHANGE W FILENAME

    std::string gr_name;

    if(inFile.is_open()){
        std::string line;
        while(std::getline(inFile,line)){
            std::stringstream ss(line);

            OLink link_;
            std::getline(ss, line, ',');
            gr_name = line;
            std::getline(ss, line, ',');
            link_.name = line;
            std::getline(ss, line, ',');
            link_.size = stdvec_to_eigen_vec(read_into_vector(line));
            std::getline(ss, line, ',');
            link_.rpy = stdvec_to_eigen_vec(read_into_vector(line));
            std::getline(ss, line, ',');
            link_.pos = stdvec_to_eigen_vec(read_into_vector(line));

            std::string joint_line;
            std::getline(ss,joint_line,';');
            std::stringstream ss2(joint_line);

            OJoints joints;
            std::vector<OJoint> joints_sub;
            bool flag = true;
            while(getline(ss2,joint_line,','))
            {
                OJoint joint;
                if(flag)
                {
                    joint.name = joint_line;

                }
                else
                {
                    joint.name = joint_line;
                }
                getline(ss2,joint_line,',');
                if(flag) // main joint, which contains the position and orientation, other joints have pos and rpy 0
                {
                    joints.pos = stdvec_to_eigen_vec(read_into_vector(joint_line));
                    getline(ss2,joint_line,',');
                    joints.rpy = stdvec_to_eigen_vec(read_into_vector(joint_line));
                    getline(ss2,joint_line,',');
                    flag = false;
                }
                else
                {
                    getline(ss2,joint_line,','); // pos 0 x or y axis rotation or prism joint
                    getline(ss2,joint_line,','); // rpy 0
                }

                if(joint_line == "prismatic")
                    joint.type = prismatic;
                else
                    joint.type = revolute;

                getline(ss2,joint_line,',');
                joint.direction = find_direction_axis(read_into_vector(joint_line));
                joints_sub.push_back(joint);
            }
            joints.joints = joints_sub;
            Object o_(gr_name,link_,joints);
            objects_.push_back(o_);
        }
    }
    inFile.close();
}

