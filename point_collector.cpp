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


#include <Eigen/Dense>
#include <robowflex_library/util.h>
#include <robowflex_dart/rotation_helper.h>

using namespace Eigen;


std::string link_name,link_size,link_xyz,link_rpy;
std::string joint_name,joint_xyz,joint_rpy;
std::vector<std::string> link_names;
std::vector<std::string> joint_names;
std::vector<std::vector<double>> link_sizes,link_positions,link_rotations;
std::vector<std::vector<double>> joint_positions,joint_rotations;

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
    return (joint_pos+ get_quaternion_from_euler(degrees[2],degrees[1],degrees[0])*(point-joint_pos));
}

MatrixXd center_joint_pos(Vector3d joint_pos, Vector3d object_xyz){
    return (joint_pos+object_xyz);
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
            // grundflaeche
            vertices.row(0), vertices.row(1)- vertices.row(0), vertices.row(2)- vertices.row(0),
            // seite links
            vertices.row(0), vertices.row(4)- vertices.row(0), vertices.row(1)- vertices.row(0),
            //seite rechts
            vertices.row(2), vertices.row(3)- vertices.row(2), vertices.row(6)- vertices.row(2),
            //seite hinten
            vertices.row(0), vertices.row(2)- vertices.row(0), vertices.row(4)- vertices.row(0),
            //seite vorne
            vertices.row(1), vertices.row(5)- vertices.row(1), vertices.row(3)- vertices.row(1),
            //seite oben
            vertices.row(4), vertices.row(5)- vertices.row(4), vertices.row(6)- vertices.row(4);
    return surface_equation;
}

MatrixXd surface_lines(MatrixXd mid_point){
    // CALC LINES BETWEEN TWO MIDDLE POINTS - HORIZONTAL AND VERTICAL FOR EACH SURFACE, HORIZONTAL (X-AXIS) FIRST, VERTICAL (Y-AXIS) SECOND
    MatrixXd surface_lines(12,6);

    surface_lines <<//  GROUND AREA
                  mid_point.row(0), mid_point.row(3)-mid_point.row(0),
            mid_point.row(1), mid_point.row(2)-mid_point.row(1),
            //  SIDE LEFT
            mid_point.row(4), mid_point.row(5)-mid_point.row(4),
            mid_point.row(0), mid_point.row(8)-mid_point.row(0),
            //  SIDE FRONT
            mid_point.row(2), mid_point.row(10)-mid_point.row(2),
            mid_point.row(5), mid_point.row(7)-mid_point.row(5),
            //  SIDE RIGHT
            mid_point.row(6), mid_point.row(7)-mid_point.row(6),
            mid_point.row(3), mid_point.row(11)-mid_point.row(3),
            //  SIDE BEHIND
            mid_point.row(1), mid_point.row(9)-mid_point.row(1),
            mid_point.row(6), mid_point.row(4)-mid_point.row(6),
            //  TOP AREA
            mid_point.row(8), mid_point.row(11)-mid_point.row(8),
            mid_point.row(9), mid_point.row(10)-mid_point.row(9);
    return surface_lines;
}

MatrixXd generate_points_from_line(MatrixXd line, int number=3){

    Vector3d point;
    Map<MatrixXd> base_vector (line.data(),1,3);
    Map<MatrixXd> direction_vector (line.data()+3,1,3);
    //std::cout << base_vector << " - " << number << std::endl;
    //std::cout << direction_vector << std::endl;
    MatrixXd points(1,number*3);

    switch (number) {
        case 1:
            return (base_vector + (direction_vector*0.5)); // middle
        case 2:
            points << base_vector+0.1* direction_vector, base_vector+0.9*direction_vector; // left side right side
            return points;
        case 3:
            points << base_vector + (direction_vector*0.5), base_vector+0.1* direction_vector, base_vector+0.9*direction_vector; // middle, left side right side
            return points;
        default:
            int n = number +1;
            int m = 0;
            for (int i = 1; i< n; i++) { //better if number %2 = 1
                double t = double(i) / double(n);
                MatrixXd point_p(1,3);
                point_p << base_vector + t * direction_vector;
                int k = 0;
                for (int j = m; j < m + 3; j++) {
                    points(0, j) = point_p(0, k);
                    k+= 1;
                }
                m += 3;
            }
            return points;
    }
}

MatrixXd generate_points_each_surface(MatrixXd lines,int number=3){

    MatrixXd points_all(12,number*3);
    for(int i = 0; i< lines.rows() ; i++){
        points_all.row(i) =  generate_points_from_line(lines.row(i),number);
    }
    return points_all;
}

MatrixXd merge_rotation(MatrixXd obj_rpy, MatrixXd joint_rpy){
    return (obj_rpy+joint_rpy);
}

MatrixXd sort_quaternion(Quaterniond q){
    MatrixXd sorted_q(1,4);
    sorted_q << q.w(), q.x(),q.y(),q.z();
    return sorted_q;
}
MatrixXd translate_rotations(MatrixXd rpy){
    MatrixXd quaternions(6,4*3);
    quaternions <<
                sort_quaternion(get_quaternion_from_euler(rpy(0),rpy(1)+M_PI*0.5,rpy(2))),  // oben
                sort_quaternion(get_quaternion_from_euler(rpy(0),rpy(1)+M_PI*0.5,rpy(2)+M_PI*0.5)),
                sort_quaternion(get_quaternion_from_euler(rpy(0),rpy(1)+M_PI*0.5,rpy(2)-M_PI*0.5)),

                sort_quaternion(get_quaternion_from_euler(rpy(2)+M_PI*0.5,rpy(1),rpy(0))),  // links
                sort_quaternion(get_quaternion_from_euler(rpy(2)+M_PI*0.5,rpy(1),rpy(0)+M_PI*0.5)),  // links
                sort_quaternion(get_quaternion_from_euler(rpy(2)+M_PI*0.5,rpy(1),rpy(0)-M_PI*0.5)),  // links

                sort_quaternion(get_quaternion_from_euler(rpy(2)-M_PI*0.5,rpy(1),rpy(0))),             // rechts
                sort_quaternion(get_quaternion_from_euler(rpy(2)-M_PI*0.5,rpy(1),rpy(0)+M_PI*0.5)),  // rechts
                sort_quaternion(get_quaternion_from_euler(rpy(2)-M_PI*0.5,rpy(1),rpy(0)-M_PI*0.5)),  // rechts

                sort_quaternion(get_quaternion_from_euler(rpy(2),rpy(1),rpy(0))),                      // vorne
                sort_quaternion(get_quaternion_from_euler(rpy(2),rpy(1),rpy(0)+M_PI*0.5)),           // vorne
                sort_quaternion(get_quaternion_from_euler(rpy(2),rpy(1),rpy(0)-M_PI*0.5)),           // vorne


                sort_quaternion(get_quaternion_from_euler(rpy(2),rpy(1)-M_PI,rpy(0))),              // hinten
                sort_quaternion(get_quaternion_from_euler(rpy(2),rpy(1)-M_PI,rpy(0)+M_PI*0.5)),      // hinten
                sort_quaternion(get_quaternion_from_euler(rpy(2),rpy(1)-M_PI,rpy(0)-M_PI*0.5)),      // hinten

                sort_quaternion(get_quaternion_from_euler(rpy(0),rpy(1)-M_PI*0.5,rpy(2))),  // unten
                sort_quaternion(get_quaternion_from_euler(rpy(0),rpy(1)-M_PI*0.5,rpy(2)+M_PI*0.5)),  // unten
                sort_quaternion(get_quaternion_from_euler(rpy(0),rpy(1)-M_PI*0.5,rpy(2)-M_PI*0.5));  // unten
    return quaternions;
}

MatrixXd create_pose(MatrixXd point, MatrixXd rotation){
    MatrixXd pose(1,7); // 3 xyz, 4 quaternion
    pose << point , rotation;
    return pose;
}
MatrixXd get_pose_all_points(){
    /*
     * TODO: for each surface -> for each point in matrix -> construct (row?) matrix containing poses / for each surface?
     * TODO: get point -> match w corresponding rotation(3x possible) (using create_pose)
     * TODO: save all poses separetely / together?
     *
     */
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



void read_txt_file(std::string filename){
    std::ifstream inFile("xml_files/doors.txt"); // CHANGE W FILENAME
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

int main() {

    /*Vector3d joint_pos;
    joint_pos << 0.8,0,0.7;
    Vector3d object_size;
    object_size << 0.03,0.3,0.2;

    //  std::cout << object_size(1) << std::endl;
    auto vertices = get_object_vertices(joint_pos,object_size);
    auto surface_equation = get_surface_equation(vertices);
    std::cout << surface_equation << std::endl;

    auto middle_po = get_middle_points(vertices);
    auto lines = surface_lines(middle_po);

    auto points = generate_points_each_surface(lines,5);

   // std::cout << points << std::endl;

    std::vector<double> rotation_deg(3);
    rotation_deg[0] = 1.57;
    rotation_deg[1] = 0;
    rotation_deg[2] = 0;
    Vector3d point;
    point << 0.77, -0.3, 0.9;
    // RPY GIVEN X Y Z ? -> MUST BE YAW Z  PITCH Y  ROLL X

   auto get_rot = get_rotated_vertex(rotation_deg,point,joint_pos);
  // std::cout << get_rot << std::endl;
    // calc object position if object xyz not 0 0 0 -> push jointpos to get correct edge points

   auto get_rotobj = get_rotated_object(rotation_deg,joint_pos,object_size);
   std::cout << get_rotobj << std::endl;

   auto s = get_quaternion_from_euler(1.57,0,1.57);

    std::cout << "Quaternionff: "  << s.coeffs() << std::endl;
    std::cout << s.w() <<" , " << s.x() <<" , "<< s.y() << " , " << s.z() << std::endl;

   auto d = quaternion_to_euler(s);

   std::cout << "euler : " << d << std::endl;

   MatrixXd rpy(1,3);
   rpy << 0.0,0.0,0.5235;
   //rpy << 0.0,0.0,0.0;
    auto res =translate_rotations(rpy);
    std::cout << "here am i" << std::endl;
    std::cout << res << std::endl;
*/


/* RUN URDF2TXT
    Py_Initialize();
    FILE *fp;
    fp = _Py_fopen("/home/serboba/PycharmProjects/urdf_extract2txt/urdf2txt.py", "r+");
    PyRun_SimpleFile(fp, "/home/serboba/PycharmProjects/urdf_extract2txt/urdf2txt.py");

    Py_Finalize();
*/
    read_txt_file("doors.txt");
    Vector3d joint_pos(joint_positions[1].data());
    Vector3d joint_rpy(joint_rotations[1].data());

    Vector3d object_size(link_sizes[1].data());
    Vector3d object_rpy (link_rotations[1].data());
    Vector3d object_xyz (link_positions[1].data());
    /*
     * TODO GET ONE PROCESS COMPLETELY DONE
     */

    return 0;
}
