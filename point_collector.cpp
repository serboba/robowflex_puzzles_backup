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
#include <robowflex_dart/Object.h>

using namespace Eigen;

std::string link_name,link_size,link_xyz,link_rpy;
std::string joint_name,joint_xyz,joint_rpy, group_name;
std::vector<std::string> link_names,joint_names,joint_names_srdf,group_names;
std::vector<std::vector<double>> link_sizes,link_positions,link_rotations;
std::vector<std::vector<double>> joint_positions,joint_rotations;

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

Quaterniond get_quaternion_from_euler(float yaw, float pitch, float roll){
    Quaterniond q;
    q =  AngleAxisd (yaw,Vector3d::UnitZ())*AngleAxisd (pitch,Vector3d::UnitY()) *AngleAxisd (roll,Vector3d::UnitX());
    return q;
}

MatrixXd quaternion_to_euler(Quaterniond q){
    return q.toRotationMatrix().eulerAngles(2,1,0);
}

MatrixXd get_rotated_vertex(std::vector<double> degrees, Vector3d point, Vector3d joint_pos ){
    /*
    std::cout << "degrees : " << degrees[0] << "-" << degrees[1] << "-" << degrees[2] << std::endl;
    std::cout<< "point : " << point << std::endl;
    std::cout<< "jopoint : " << joint_pos << std::endl;
    std::cout << "quat valw : " << get_quaternion_from_euler(degrees[2],degrees[1],degrees[0]).w() << std::endl;
    std::cout << "quat valx : " << get_quaternion_from_euler(degrees[2],degrees[1],degrees[0]).x() << std::endl;
    std::cout << "quat valy : " << get_quaternion_from_euler(degrees[2],degrees[1],degrees[0]).y() << std::endl;
    std::cout << "quat valz : " << get_quaternion_from_euler(degrees[2],degrees[1],degrees[0]).z() << std::endl;
    */
    return (joint_pos+ get_quaternion_from_euler(degrees[2],degrees[1],degrees[0])*(point-joint_pos));
}

MatrixXd get_rotated_object(std::vector<double> degrees, Vector3d joint_pos, Vector3d object_size){ //rotate for all vertices
    MatrixXd vertices = get_object_vertices(joint_pos,object_size);
    for(int i = 0 ; i< vertices.rows(); i++){
        Vector3d row = vertices.row(i);
        MatrixXd new_v = get_rotated_vertex(degrees, row, joint_pos);
        vertices.row(i) << new_v(0),new_v(1),new_v(2);
    }
    return vertices;
}

MatrixXd get_middle_points(MatrixXd vertices){
    MatrixXd edge_middle(12,3);
    //unten
    edge_middle <<
                (vertices.row(0)+vertices.row(1))/2, // links zu +x-achse
            (vertices.row(0)+vertices.row(2))/2, // links zu +y-achse
            (vertices.row(1)+vertices.row(3))/2, // links vorne +y achse
            (vertices.row(2)+vertices.row(3))/2, // rechs unten x achse
            //seiten
            (vertices.row(0)+vertices.row(4))/2, //links hinten +z
            (vertices.row(1)+vertices.row(5))/2, // links vorne +z
            (vertices.row(2)+vertices.row(6))/2, // rechts hinten +z
            (vertices.row(3)+vertices.row(7))/2, // rechts vorne +z
            //oben
            (vertices.row(4)+vertices.row(5))/2, // rechts zu x+ achse
            (vertices.row(4)+vertices.row(6))/2, // rechts zu y+ achse
            (vertices.row(5)+vertices.row(7))/2, // rechts vorne y+ achse
            (vertices.row(6)+vertices.row(7))/2; // rechts x achse

    return edge_middle;
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
            if(abs(normals(i,j)) < 0.001){
                normals(i,j) = 0.0;
                continue;
            }
            if(normals(i,j) > 0.0){
                normals(i,j) = 0.01;
            }else{
                normals(i,j) = -0.01;
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
    //std::cout << "NORMALS BEFORE ADJUST" << std::endl;
    //std::cout << normals << std::endl;
    return adjust_normals(normals);
}

MatrixXd sort_quaternion(Quaterniond q){
    MatrixXd sorted_q(1,4);
    sorted_q << q.w(), q.x(),q.y(),q.z();
    return sorted_q;
}

MatrixXd translate_rotations(MatrixXd rpy){
    MatrixXd quaternions(6,4*3);
    quaternions <<
                sort_quaternion(get_quaternion_from_euler(rpy(0),rpy(1)-M_PI*0.5,rpy(2))),  // unten
            sort_quaternion(get_quaternion_from_euler(rpy(0),rpy(1)-M_PI*0.5,rpy(2)+M_PI*0.5)),  // unten
            sort_quaternion(get_quaternion_from_euler(rpy(0),rpy(1)-M_PI*0.5,rpy(2)-M_PI*0.5)),  // unten

            sort_quaternion(get_quaternion_from_euler(rpy(2)+M_PI*0.5,rpy(1),rpy(0))),  // links
            sort_quaternion(get_quaternion_from_euler(rpy(2)+M_PI*0.5,rpy(1),rpy(0)+M_PI*0.5)),  // links
            sort_quaternion(get_quaternion_from_euler(rpy(2)+M_PI*0.5,rpy(1),rpy(0)-M_PI*0.5)),  // links


            sort_quaternion(get_quaternion_from_euler(rpy(2),rpy(1)-M_PI,rpy(0))),              // hinten
            sort_quaternion(get_quaternion_from_euler(rpy(2),rpy(1)-M_PI,rpy(0)+M_PI*0.5)),      // hinten
            sort_quaternion(get_quaternion_from_euler(rpy(2),rpy(1)-M_PI,rpy(0)-M_PI*0.5)),      // hinten

            sort_quaternion(get_quaternion_from_euler(rpy(2)-M_PI*0.5,rpy(1),rpy(0))),             // rechts
            sort_quaternion(get_quaternion_from_euler(rpy(2)-M_PI*0.5,rpy(1),rpy(0)+M_PI*0.5)),  // rechts
            sort_quaternion(get_quaternion_from_euler(rpy(2)-M_PI*0.5,rpy(1),rpy(0)-M_PI*0.5)),  // rechts

            sort_quaternion(get_quaternion_from_euler(rpy(2),rpy(1),rpy(0))),                      // vorne
            sort_quaternion(get_quaternion_from_euler(rpy(2),rpy(1),rpy(0)+M_PI*0.5)),           // vorne
            sort_quaternion(get_quaternion_from_euler(rpy(2),rpy(1),rpy(0)-M_PI*0.5)),           // vorne


            sort_quaternion(get_quaternion_from_euler(rpy(0),rpy(1)+M_PI*0.5,rpy(2))),  // oben
            sort_quaternion(get_quaternion_from_euler(rpy(0),rpy(1)+M_PI*0.5,rpy(2)+M_PI*0.5)),
            sort_quaternion(get_quaternion_from_euler(rpy(0),rpy(1)+M_PI*0.5,rpy(2)-M_PI*0.5));

    return quaternions;
}

MatrixXd new_rotation_quaternion(MatrixXd rpy, int surface_no){ // rpy sum = actual rot + wanted rot

    switch (surface_no) {
        case 0: return (sort_quaternion(get_quaternion_from_euler(rpy(0),rpy(1)-M_PI*0.5,rpy(2))));             // unten
        case 1: return (sort_quaternion(get_quaternion_from_euler(rpy(0),rpy(1)-M_PI*0.5,rpy(2)+M_PI*0.5)));         // unten
        case 2: return (sort_quaternion(get_quaternion_from_euler(rpy(0),rpy(1)-M_PI*0.5,rpy(2)-M_PI*0.5)));         // unten

        case 3: return (sort_quaternion(get_quaternion_from_euler(rpy(2)+M_PI*0.5,rpy(1),rpy(0))));                  // links
        case 4: return (sort_quaternion(get_quaternion_from_euler(rpy(2)+M_PI*0.5,rpy(1),rpy(0)+M_PI*0.5)));         // links
        case 5: return (sort_quaternion(get_quaternion_from_euler(rpy(2)+M_PI*0.5,rpy(1),rpy(0)-M_PI*0.5)));         // links


        case 6: return (sort_quaternion(get_quaternion_from_euler(rpy(2),rpy(1)-M_PI,rpy(0))));                      // hinten
        case 7: return (sort_quaternion(get_quaternion_from_euler(rpy(2),rpy(1)-M_PI,rpy(0)+M_PI*0.5)));             // hinten
        case 8: return (sort_quaternion(get_quaternion_from_euler(rpy(2),rpy(1)-M_PI,rpy(0)-M_PI*0.5)));             // hinten

        case 9: return (sort_quaternion(get_quaternion_from_euler(rpy(2)-M_PI*0.5,rpy(1),rpy(0))));                  // rechts
        case 10: return (sort_quaternion(get_quaternion_from_euler(rpy(2)-M_PI*0.5,rpy(1),rpy(0)+M_PI*0.5)));        // rechts
        case 11: return (sort_quaternion(get_quaternion_from_euler(rpy(2)-M_PI*0.5,rpy(1),rpy(0)-M_PI*0.5)));        // rechts

        case 12: return (sort_quaternion(get_quaternion_from_euler(rpy(2),rpy(1),rpy(0))));                          // vorne
        case 13: return (sort_quaternion(get_quaternion_from_euler(rpy(2),rpy(1),rpy(0)+M_PI*0.5)));                 // vorne
        case 14: return (sort_quaternion(get_quaternion_from_euler(rpy(2),rpy(1),rpy(0)-M_PI*0.5)));                 // vorne


        case 15: return (sort_quaternion(get_quaternion_from_euler(rpy(0),rpy(1)+M_PI*0.5,rpy(2))));          // oben
        case 16: return (sort_quaternion(get_quaternion_from_euler(rpy(0),rpy(1)+M_PI*0.5,rpy(2)-M_PI*0.5)));   // oben
        case 17: return (sort_quaternion(get_quaternion_from_euler(rpy(0),rpy(1)+M_PI*0.5,rpy(2)+M_PI*0.5)));   // oben
    }
}


std::vector<double> read_into_vector(std::string str){

    std::istringstream ss(str);
    std::string s;
    std::vector<double> vec;
    while(std::getline(ss,s,' ')){
        double temp = std::stod(s);
        vec.push_back(temp);
    }
    return vec;
}

void read_srdf_file(std::string filename){
    std::string str = "xml_files/" + filename;
    std::ifstream inFile(str); // CHANGE W FILENAME
    if(inFile.is_open()){
        std::string line;
        int i =0;
        while(std::getline(inFile,line)){
            std::stringstream ss(line);
            std::getline(ss, joint_name, ',');
            joint_names_srdf.push_back(joint_name);

            std::getline(ss, group_name, ',');
            group_names.push_back(group_name);
        }
    }
    inFile.close();
}

void read_urdf_file(std::string filename){
    std::string str = "xml_files/" + filename;
    std::ifstream inFile(str); // CHANGE W FILENAME
    if(inFile.is_open()){
        std::string line;
        int i =0;
        while(std::getline(inFile,line)){
            std::stringstream ss(line);
            if(i ==0) {
                std::getline(ss, link_name, ',');
                link_names.push_back(link_name);

                std::getline(ss, link_size, ',');
                link_sizes.push_back(read_into_vector(link_size));

                std::getline(ss, link_rpy, ',');
                link_rotations.push_back(read_into_vector(link_rpy));

                std::getline(ss, link_xyz, ',');
                link_positions.push_back(read_into_vector(link_xyz));
                i =1;
            }else{
                std::getline(ss, joint_name, ',');
                joint_names.push_back(joint_name);

                std::getline(ss, joint_rpy, ',');
                joint_rotations.push_back(read_into_vector(joint_rpy));

                std::getline(ss, joint_xyz, ',');
                joint_positions.push_back(read_into_vector(joint_xyz));
                i =0;

            }
        }
    }
    inFile.close();
}

void create_objects_from_urdf(){
    read_srdf_file("cube_scene_srdf.txt");
    read_urdf_file("cube_scene.txt");
    //std::vector<Object> objects_;
    for(int i = 0; i < joint_positions.size(); i++){
        Vector3d joint_xyz(joint_positions[i].data());
        Vector3d joint_rpy(joint_rotations[i].data());

        Vector3d object_size(link_sizes[i].data());
        Vector3d object_rpy (link_rotations[i].data());
        Vector3d object_xyz (link_positions[i].data());

        std::string object_gr_name = group_names[i].data();
        std::string joint_name = joint_names[i].data();
        std::string link_name = link_names[i].data();
        objects.push_back(Object(link_name,object_size,object_xyz,object_rpy,joint_name,joint_xyz,joint_rpy,object_gr_name));
    }
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
    }
    return ratio;
}

MatrixXd get_point_position(MatrixXd surface_equation,std::vector<double> ratio_dir_vectors){

    Map<MatrixXd> position_vector(surface_equation.data(),1,3);
    Map<MatrixXd> direction_vector1(surface_equation.data()+3,1,3);
    Map<MatrixXd> direction_vector2(surface_equation.data()+6,1,3);
    direction_vector1 *= ratio_dir_vectors[0];
    direction_vector2 *= ratio_dir_vectors[1];

    return (position_vector+direction_vector1+direction_vector2);
}

MatrixXd get_random_point_from_surface(MatrixXd surface_equation_matrix, MatrixXd rpy_matrix, MatrixXd normals){ // eine ebenengleichung
    srand ( time(NULL) );
    int surface_no =rand() %6; // SURFACE NUMBER-> NEED IT FOR QUATERNIONS !!! DONT LOSE IT
    int quaternion_no = rand() %3;
    int surf_q_no = (surface_no*3)+quaternion_no; // surf no w quats
    std::cout << " SURFACE NO: " << surface_no << std::endl;
    std::cout << " QUAT NO: " << quaternion_no << std::endl;

    MatrixXd surface_equation = extract_surface(surface_no, surface_equation_matrix);
    MatrixXd quaternion = match_quaternion_to_surface(quaternion_no,surface_no,rpy_matrix);

    MatrixXd position(1,11);
    position << get_point_position(surface_equation,get_ratio_dir_vecs()),quaternion,normals.row(surface_no), surf_q_no;
    return position;

}

std::vector<double> eigenvector_to_std(Vector3d vec){
    std::vector<double> degree;
    degree.resize(vec.size());
    VectorXd::Map(&degree[0],vec.size()) = vec;
    return degree;
}

MatrixXd vec_to_matrix(Vector3d vec){
    MatrixXd m(1,3);
    m << vec[0],vec[1],vec[2];
    return m;
}

Eigen::MatrixXd get_pose_object(Object &obj){

    std::vector<double> degrees = eigenvector_to_std(obj.actual_rotation);
    MatrixXd actual_rpy = vec_to_matrix(obj.actual_rotation);
    obj.actual_position = get_rotated_vertex(degrees, (obj.link_xyz + obj.joint_xyz), obj.joint_xyz); // ADJUST JOINT POS W RESPECT TO LINK XYZ

    std::cout <<  "Actual position : " << obj.actual_position << std::endl;
    std::cout <<  "Actual rotation : " << actual_rpy << std::endl;

    MatrixXd surface_equation = get_surface_equation(get_rotated_object(degrees, obj.actual_position,obj.link_size));
    MatrixXd normals = get_normal_of_plane(surface_equation);
    MatrixXd random_pose = get_random_point_from_surface(surface_equation,actual_rpy,normals);

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
    Py_Initialize();
    FILE *fp;
    fp = _Py_fopen("/home/serboba/PycharmProjects/urdf_extract2txt/urdf2txt.py", "r+");
    PyRun_SimpleFile(fp, "/home/serboba/PycharmProjects/urdf_extract2txt/urdf2txt.py");
    Py_Finalize();

}

