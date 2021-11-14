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
    std::cout << "degrees : " << degrees[0] << "-" << degrees[1] << "-" << degrees[2] << std::endl;
    std::cout<< "point : " << point << std::endl;
    std::cout<< "jopoint : " << joint_pos << std::endl;
    std::cout << "quat valw : " << get_quaternion_from_euler(degrees[2],degrees[1],degrees[0]).w() << std::endl;
    std::cout << "quat valx : " << get_quaternion_from_euler(degrees[2],degrees[1],degrees[0]).x() << std::endl;
    std::cout << "quat valy : " << get_quaternion_from_euler(degrees[2],degrees[1],degrees[0]).y() << std::endl;
    std::cout << "quat valz : " << get_quaternion_from_euler(degrees[2],degrees[1],degrees[0]).z() << std::endl;

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
    std::cout << "SURFACE:  " << surface_equation_matrix << std::endl;
    std::cout << "SURFACE:  " << surface_equation_matrix.row(surface_no) << std::endl;
    std::cout << "SURFACE NO: " << surface_no << std::endl;
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
        case 0: return (sort_quaternion(get_quaternion_from_euler(rpy(0),rpy(1)-M_PI*0.5,rpy(2))));     break;        // unten
        case 1: return (sort_quaternion(get_quaternion_from_euler(rpy(0),rpy(1)-M_PI*0.5,rpy(2)+M_PI*0.5))); break;        // unten
        case 2: return (sort_quaternion(get_quaternion_from_euler(rpy(0),rpy(1)-M_PI*0.5,rpy(2)-M_PI*0.5))); break;        // unten

        case 3: return (sort_quaternion(get_quaternion_from_euler(rpy(2)+M_PI*0.5,rpy(1),rpy(0))));          break;        // links
        case 4: return (sort_quaternion(get_quaternion_from_euler(rpy(2)+M_PI*0.5,rpy(1),rpy(0)+M_PI*0.5))); break;        // links
        case 5: return (sort_quaternion(get_quaternion_from_euler(rpy(2)+M_PI*0.5,rpy(1),rpy(0)-M_PI*0.5))); break;        // links


        case 6: return (sort_quaternion(get_quaternion_from_euler(rpy(2),rpy(1)-M_PI,rpy(0))));              break;        // hinten
        case 7: return (sort_quaternion(get_quaternion_from_euler(rpy(2),rpy(1)-M_PI,rpy(0)+M_PI*0.5)));     break;        // hinten
        case 8: return (sort_quaternion(get_quaternion_from_euler(rpy(2),rpy(1)-M_PI,rpy(0)-M_PI*0.5)));     break;        // hinten

        case 9: return (sort_quaternion(get_quaternion_from_euler(rpy(2)-M_PI*0.5,rpy(1),rpy(0))));           break;       // rechts
        case 10: return (sort_quaternion(get_quaternion_from_euler(rpy(2)-M_PI*0.5,rpy(1),rpy(0)+M_PI*0.5))); break;       // rechts
        case 11: return (sort_quaternion(get_quaternion_from_euler(rpy(2)-M_PI*0.5,rpy(1),rpy(0)-M_PI*0.5))); break;       // rechts

        case 12: return (sort_quaternion(get_quaternion_from_euler(rpy(2),rpy(1),rpy(0))));                   break;       // vorne
        case 13: return (sort_quaternion(get_quaternion_from_euler(rpy(2),rpy(1),rpy(0)+M_PI*0.5)));          break;       // vorne
        case 14: return (sort_quaternion(get_quaternion_from_euler(rpy(2),rpy(1),rpy(0)-M_PI*0.5)));          break;       // vorne


        case 15: return (sort_quaternion(get_quaternion_from_euler(rpy(0),rpy(1)+M_PI*0.5,rpy(2))));  break;          // oben
        case 16: return (sort_quaternion(get_quaternion_from_euler(rpy(0),rpy(1)+M_PI*0.5,rpy(2)-M_PI*0.5))); break;  // oben
        case 17: return (sort_quaternion(get_quaternion_from_euler(rpy(0),rpy(1)+M_PI*0.5,rpy(2)+M_PI*0.5)));  break; // oben

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

MatrixXd get_random_point_from_surface(MatrixXd surface_equation_matrix, MatrixXd joint_rpy_matrix, MatrixXd normals){ // eine ebenengleichung
    srand ( time(NULL) );

    int surface_no =rand() %6; // SURFACE NUMBER-> NEED IT FOR QUATERNIONS !!! DONT LOSE IT
    int quaternion_no = rand() %3;

    int surf_q_no = (surface_no*3)+quaternion_no;
    std::cout << " SURFACE NO: " << surface_no << std::endl;
    std::cout << " QUU NO: " << quaternion_no << std::endl;
    MatrixXd surface_equation = extract_surface(surface_no, surface_equation_matrix);

    MatrixXd quaternions = translate_rotations(joint_rpy_matrix); // calculate corresponding quaternions for pose

    // quat row for obj is surface_no*3 + quaternion no
    MatrixXd  q = quaternions.row(surface_no).block(0,quaternion_no*4,1,4);

    std::cout << "SURF :: " << surface_equation << std::endl;
    Quaterniond q1;
    q1.w() = q(0);
    q1.x() = q(1);
    q1.y() = q(2);
    q1.z() = q(3);
    std::cout << "QUAT DEGREE :: " << quaternion_to_euler(q1) << std::endl;
    srand ( time(NULL) );

    double ratio_vec1 = (rand() %8);
    double ratio_vec2 = (rand() %8);

    std::cout << "START" << std::endl;

    if(ratio_vec1== 0 || ratio_vec1== 1|| ratio_vec1== 2) // EDGES WAY TOO HARD TO GRASP AND MOSTLY INACCURATE SO MAKE IT TO THE MIDDLE
        ratio_vec1 = 5;
    if(ratio_vec2 == 0 || ratio_vec2== 1|| ratio_vec2== 2)
        ratio_vec2 = 5;

    Map<MatrixXd> position_vector(surface_equation.data(),1,3);
    Map<MatrixXd> direction_vector1(surface_equation.data()+3,1,3);
    Map<MatrixXd> direction_vector2(surface_equation.data()+6,1,3);

    std::cout << position_vector << std::endl;
    std::cout << direction_vector1 << std::endl;
    std::cout << direction_vector2 << std::endl;
    std::cout << "-> MULTIPLY" << std::endl;

    direction_vector1 *= ratio_vec1/10;
    direction_vector2 *= ratio_vec2/10;

    std::cout << direction_vector1 << std::endl;
    std::cout << direction_vector2 << std::endl;


    std::cout << "NORMALS ROW FOR SURFACE : " << surface_no<< std::endl;
    std::cout << normals.row(surface_no) << std::endl;



    MatrixXd position(1,11);
    position << position_vector+direction_vector1+direction_vector2,q,normals.row(surface_no), surf_q_no;
    std::cout << position << std::endl;

    return position;

}


Eigen::MatrixXd get_pose_object(Object &obj){

    std::vector<double> degr;
    degr.resize(obj.actual_rotation.size());
    VectorXd::Map(&degr[0],obj.actual_rotation.size()) = obj.actual_rotation;

    MatrixXd joint_rpy_matrix(1,3);
    joint_rpy_matrix << obj.actual_rotation[0],obj.actual_rotation[1],obj.actual_rotation[2];
    obj.actual_position = get_rotated_vertex(degr, (obj.link_xyz + obj.joint_xyz), obj.joint_xyz); // ADJUST JOINT POS W RESPECT TO LINK XYZ

    std::cout <<  "HOPPPP acc : " << obj.actual_position << std::endl;
    std::cout <<  "HOPPPP jj : " << joint_rpy_matrix << std::endl;

    MatrixXd vertices = get_rotated_object(degr, obj.actual_position,obj.link_size);
    MatrixXd surface_equation = get_surface_equation(vertices);
    MatrixXd normals = get_normal_of_plane(surface_equation);
    MatrixXd random_pose = get_random_point_from_surface(surface_equation,joint_rpy_matrix,normals);

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

