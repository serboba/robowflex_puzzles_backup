//
// Created by serboba on 21.10.21.
//
//
// Created by serboba on 21.10.21.
//

#include <Python.h>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <random>
#include <Eigen/Dense>
#include <robowflex_library/util.h>
#include <robowflex_dart/point_collector.h>
#include <robowflex_dart/quaternion_factory.h>
#include <robowflex_dart/conversion_functions.h>
#include <robowflex_dart/Object.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace Eigen;
/*
std::string link_name,link_size,link_xyz,link_rpy;
std::string joint_name,joint_xyz,joint_rpy, group_name, joint_type, joint_axis, joint_bound;
std::vector<std::string> link_names,joint_names,joint_names_srdf,group_names, joint_types;
std::vector<std::vector<double>> link_sizes,link_positions,link_rotations;
std::vector<std::vector<double>> joint_positions,joint_rotations, joint_axes, joint_bounds;
*/
std::vector<Object> objects;

MatrixXd get_object_vertices(Vector3d joint_pos, Vector3d object_size){
    MatrixXd vertices(8,3);
    vertices <<
             //unten
            joint_pos.x()-object_size.x(), joint_pos.y()-object_size.y(), joint_pos.z()-object_size.z(), //links hinten    0
            joint_pos.x()+object_size.x(), joint_pos.y()-object_size.y(), joint_pos.z()-object_size.z(), //links vorne     1
            joint_pos.x()-object_size.x(), joint_pos.y()+object_size.y(), joint_pos.z()-object_size.z(), // rechts hinten  2
            joint_pos.x()+object_size.x(), joint_pos.y()+object_size.y(), joint_pos.z()-object_size.z(), // rechts vorne   3

            //oben
            joint_pos.x()-object_size.x(), joint_pos.y()-object_size.y(), joint_pos.z()+object_size.z(), //links hinten    4
            joint_pos.x()+object_size.x(), joint_pos.y()-object_size.y(), joint_pos.z()+object_size.z(), //links vorne     5
            joint_pos.x()-object_size.x(), joint_pos.y()+object_size.y(), joint_pos.z()+object_size.z(), //rechts hinten   6
            joint_pos.x()+object_size.x(), joint_pos.y()+object_size.y(), joint_pos.z()+object_size.z(); // rechts vorne   7
    return vertices;
}


MatrixXd get_rotated_vertex(Vector3d obj_rpy, Vector3d point, Vector3d joint_pos ){ // todo correct quaternion value with divide rpy

    MatrixXd q1 = rpy_to_quaternion(obj_rpy[0],obj_rpy[1],obj_rpy[2]);
    return (joint_pos + matrix_to_quaternion(q1)*(point-joint_pos));
}
/*
MatrixXd get_rotated_vertex(MatrixXd q1, Vector3d point, Vector3d joint_pos ){ // todo correct quaternion value with divide rpy

    return (joint_pos + matrix_to_quaternion(q1)*(point-joint_pos));
    //return (joint_pos+ get_quaternion_from_euler(degrees[2],degrees[1],degrees[0])*(point-joint_pos));
}

MatrixXd get_rotated_vertex(MatrixXd quat, Vector3d new_rpy, Vector3d point, Vector3d joint_pos , int surf_no ){ // todo correct quaternion value with divide rpy

    //MatrixXd q1 = rpy_to_quaternion(obj_rpy[0],obj_rpy[1],obj_rpy[2]);
    MatrixXd deg = vec_to_matrix(new_rpy);
    MatrixXd q1 = new_rotation_quaternion(deg, matrix_to_quaternion(quat),0);
    switch (surf_no) {
    }

    return (joint_pos + matrix_to_quaternion(q1)*(point-joint_pos));
    //return (joint_pos+ get_quaternion_from_euler(degrees[2],degrees[1],degrees[0])*(point-joint_pos));
}

*/
MatrixXd get_rotated_object(Vector3d obj_rpy, Vector3d joint_pos, Vector3d object_size){ //rotate for all vertices
    MatrixXd vertices = get_object_vertices(joint_pos,object_size);
    for(int i = 0 ; i< vertices.rows(); i++){
        Vector3d row = vertices.row(i);
        MatrixXd new_v = get_rotated_vertex(obj_rpy, row, joint_pos);
        vertices.row(i) << new_v(0),new_v(1),new_v(2);
    }
    return vertices;
}


MatrixXd get_surface_equation(MatrixXd vertices){
    MatrixXd surface_equation(6,9);
    surface_equation <<
            vertices.row(0), vertices.row(1)- vertices.row(0), vertices.row(2)- vertices.row(0), // grundflaeche
            vertices.row(0), vertices.row(4)- vertices.row(0), vertices.row(1)- vertices.row(0), // seite links
            vertices.row(1), vertices.row(5)- vertices.row(1), vertices.row(3)- vertices.row(1), //seite vorne
            vertices.row(2), vertices.row(3)- vertices.row(2), vertices.row(6)- vertices.row(2), //seite rechts
            vertices.row(0), vertices.row(2)- vertices.row(0), vertices.row(4)- vertices.row(0), //seite hinten
            vertices.row(4), vertices.row(5)- vertices.row(4), vertices.row(6)- vertices.row(4); //seite oben
    return surface_equation;
}

MatrixXd extract_surface(int surface_no, MatrixXd surface_equation_matrix){
    return surface_equation_matrix.row(surface_no);
}

MatrixXd adjust_normals(MatrixXd normals){
    for(int i = 0; i < normals.rows() ; i++){
        for(int j = 0; j< normals.cols() ; j++){
            if(abs(normals(i,j)) < 0.01){
                normals(i,j) = 0.0;
            }else{
                if(normals(i,j) > 0.0){
                    normals(i,j) = 0.01;
                }else{
                    normals(i,j) = -0.01;
                }
            }
        }
    }
    return normals;
}

MatrixXd get_normal_of_plane(MatrixXd surface_equation){
    Map<MatrixXd> direction_vector1(surface_equation.data()+6*3,6,3);
    Map<MatrixXd> direction_vector2(surface_equation.data()+6*6,6,3);
    MatrixXd normals(6,3);
    for(int i =0; i < direction_vector1.rows() ; i++){
        Vector3d v1 = Vector3d(direction_vector2.row(i));
        Vector3d v2 = Vector3d(direction_vector1.row(i));
        MatrixXd n_v (1,3);
        n_v = MatrixXd(v1.cross(v2).transpose());
        if(i== 5){
            n_v *= -1;
        } // top surface normal always the opposite

        normals.row(i) << n_v;
    }
    return adjust_normals(normals);
}


MatrixXd sort_quaternion(MatrixXd rpy){ // ROLL =
    MatrixXd quaternions(6,4);
    if (rpy(0) == 0.0 && rpy(1) == 0.0 && rpy(2) == 0.0){
        quaternions<< get_quaternion_unten(rpy),get_quaternion_links(rpy),get_quaternion_hinten(rpy), get_quaternion_rechts(rpy), get_quaternion_vorne(rpy), get_quaternion_oben(rpy);
        return quaternions;
    }
    quaternions <<  quaternion_x_y_z(rpy);
    return quaternions;

}

MatrixXd translate_rotations(MatrixXd rpy){
     return sort_quaternion(rpy);
}

MatrixXd new_rotation_quaternion(MatrixXd rpy,Eigen::Quaterniond q_original, int surface_no){ // rpy sum = actual rot + wanted rot
    tf2::Quaternion q_rot;
    std::vector<MatrixXd> d_ = rpy_to_vector(rpy);
    tf2::Quaternion q_org = eigen_to_tfquaternion(q_original);
    tf2::Quaternion q_new;
    tf2::Quaternion q_temp = eigen_to_tfquaternion(q_original);

    for(int i = 0; i< d_.size() ; i++){
        q_rot = eigen_to_tfquaternion(rpy_to_quaternion(d_[i](0),d_[i](1),d_[i](2)));             //unten
        q_new = q_rot*q_temp;
        q_temp= q_rot;
    }
    //q_new.normalize();
    return tf_to_eigen_matrix_q(q_new);
}


Eigen::Quaterniond rotate_quaternion(double roll, double pitch, double yaw, Quaterniond old_q){
    tf2::Quaternion q_rot;
    q_rot.setRPY(roll,pitch,yaw);
    tf2::Quaternion q_old = eigen_to_tfquaternion(old_q);

    tf2::Quaternion q_new;
    q_new = q_rot*q_old;
    //q_new.normalize();
    Eigen::Quaterniond q_res =tf_to_eigen_quaternion(q_new);
    return q_res;
}

std::vector<double> get_goal_from_urdf(std::string filename) {
    std::ifstream file_(filename);
    std::string line;
    bool found = false;
    std::string str = "<joint name=\"goal\" type=\"fixed\">";
    std::string delim = "xyz=\"";
    while(std::getline(file_,line)){
        if(line.find(str,0) != std::string::npos){
            break;
        }
    }
    std::getline(file_,line);

    size_t pos_start = 0;
    size_t pos_end, delim_len = delim.length();
    std::string token;

    if((pos_end = line.find(delim,pos_start))!= std::string::npos){
        pos_start = pos_end + delim_len;
        token = line.substr(pos_start,pos_end-pos_start);
    }
    std::vector<double> pose = read_into_vector(token);
    std::vector<double> goal_rot = {1.0,0.0,0.0,0.0}; // identity rotation
    pose.insert(pose.end(),goal_rot.begin(),goal_rot.end());

    return pose;
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
    std::string str = "obj_txt/" + filename+".txt";
    std::ifstream inFile(str); // CHANGE W FILENAME

    std::string gr_name;

    if(inFile.is_open()){
        std::string line;
        int i =0;
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


void create_objects_from_txt(const std::string &filename, std::vector<Object> &objects_){
    read_obj_txt_file(filename,objects_);

}

MatrixXd match_quaternion_to_surface(int quaternion_no, int surface_no, MatrixXd rpy_matrix){
    MatrixXd quaternions = translate_rotations(rpy_matrix); // calculate corresponding quaternions for pose (all poses)
    MatrixXd  q = quaternions.row(surface_no).block(0,quaternion_no*4,1,4); // quat row for obj is surface_no*3 + quaternion no
    return q;
}

std::vector<double> get_ratio_dir_vecs(){
    std::vector<double> ratio;
    srand ( time(NULL) );
    int i =0;
    while(i<2){ // eine ebene hat zwei richtungsvektoren also 2 mal
        double ratio_vec = (rand() %8); // ab 80% viel zu weit aussen der punkt
        if(ratio_vec== 0 || ratio_vec== 1|| ratio_vec== 2) // EDGES WAY TOO HARD TO GRASP AND MOSTLY INACCURATE SO MAKE IT TO THE MIDDLE
            ratio_vec = 5;
        ratio_vec /= 10;
        ratio.push_back(ratio_vec);
        i++;
      //  std::cout <<"RANDOM RATIO VALUE : "<< ratio_vec << std::endl;
    }
    return ratio;
}

MatrixXd get_point_position(MatrixXd surface_equation,std::vector<double> ratio_dir_vectors){

    Map<MatrixXd> position_vector(surface_equation.data(),1,3);
    Map<MatrixXd> direction_vector1(surface_equation.data()+3,1,3);
    Map<MatrixXd> direction_vector2(surface_equation.data()+6,1,3);
    direction_vector1 *= ratio_dir_vectors[0];
    direction_vector2 *= ratio_dir_vectors[1];
//    std::cout <<  "RATIO DIR VECTOR 1 " << ratio_dir_vectors[0] << std::endl;
//    std::cout <<  "RATIO DIR VECTOR 2 " << ratio_dir_vectors[1] << std::endl;

    return (position_vector+direction_vector1+direction_vector2);
}

MatrixXd get_random_point_from_surface(MatrixXd surface_equation_matrix, MatrixXd rpy_matrix, MatrixXd normals, int surface_no){ // eine ebenengleichung
    srand ( time(NULL) );
   // int surface_no =rand() %2; // SURFACE NUMBER-> NEED IT FOR QUATERNIONS !!! DONT LOSE IT
    std::vector<int> surfs;
    //surfs.push_back(1); // links
    surfs.push_back(4); // vorne
    //surfs.push_back(3); // rechts
    surfs.push_back(5); // oben
   // surface_no = surfs[surface_no];
    //int surface_no =5; // SURFACE NUMBER-> NEED IT FOR QUATERNIONS !!! DONT LOSE IT
    // (TODO) CHOOSED TOP SURFACE TO FASTER SOLUTION !
    int quaternion_no = 0; // todo +2 roll variations
    int surf_q_no = (surface_no*3)+quaternion_no; // surf no w quats
//    std::cout << " SURFACE NO: " << surface_no << std::endl;
//    std::cout << " QUAT NO: " << quaternion_no << std::endl;

    MatrixXd surface_equation = extract_surface(surface_no, surface_equation_matrix);
    MatrixXd quaternion = match_quaternion_to_surface(quaternion_no,surface_no,rpy_matrix);

    MatrixXd position(1,11);
    position << get_point_position(surface_equation,get_ratio_dir_vecs()),quaternion,normals.row(surface_no), surface_no;
    return position;

}

Eigen::MatrixXd get_pose_object(Object &obj, int surface_no){

    auto xd = obj.link.pos + obj.joints.pos;
    std::cout << "link pos : " << obj.link.pos << std::endl;
    std::cout << "joint pos : " << obj.joints.pos << std::endl;
    std::cout << "added:" << xd <<std::endl;
    std::cout << "actual pos : " << obj.actual_position << std::endl;
    auto actual_position = get_rotated_vertex(obj.actual_rotation, (obj.actual_position), obj.joints.pos); // ADJUST JOINT POS W RESPECT TO LINK XYZ
    std::cout <<  "Actual position : " << obj.actual_position << std::endl;
    std::cout <<  "Actual rotation : " << obj.actual_rotation << std::endl;
    auto vertices = get_rotated_object(obj.actual_rotation, actual_position,obj.link.size);
    MatrixXd surface_equation = get_surface_equation(get_rotated_object(obj.actual_rotation, actual_position,obj.link.size));
    MatrixXd normals = get_normal_of_plane(surface_equation);
    MatrixXd rpy_matrix= vec_to_matrix(obj.link.rpy+obj.joints.rpy);
    MatrixXd random_pose = get_random_point_from_surface(surface_equation,rpy_matrix,normals, surface_no);

    return random_pose;
}

std::vector<Object> get_objects(){
    return objects;
}

void set_objects(std::vector<Object> objects_){
    objects = objects_;
}


void create_txt_from_urdf(){
    // RUN URDF2TXT
    std::system("python urdf2txt.py");

}

