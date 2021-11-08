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

using namespace Eigen;

std::string link_name,link_size,link_xyz,link_rpy;
std::string joint_name,joint_xyz,joint_rpy, group_name;
std::vector<std::string> link_names,joint_names,joint_names_srdf,group_names;
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
            vertices.row(0), vertices.row(1)- vertices.row(0), vertices.row(2)- vertices.row(0), // grundflaeche
            vertices.row(0), vertices.row(4)- vertices.row(0), vertices.row(1)- vertices.row(0), // seite links
            vertices.row(1), vertices.row(5)- vertices.row(1), vertices.row(3)- vertices.row(1), //seite vorne
            vertices.row(2), vertices.row(3)- vertices.row(2), vertices.row(6)- vertices.row(2), //seite rechts
            vertices.row(0), vertices.row(2)- vertices.row(0), vertices.row(4)- vertices.row(0), //seite hinten
            vertices.row(4), vertices.row(5)- vertices.row(4), vertices.row(6)- vertices.row(4); //seite oben
    return surface_equation;
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

    std::cout << "NORMALS BEFORE ADJUST" << std::endl;
    std::cout << normals << std::endl;
    return adjust_normals(normals);
}

MatrixXd surface_lines(MatrixXd mid_point){
    // CALC LINES BETWEEN TWO MIDDLE POINTS - HORIZONTAL AND VERTICAL FOR EACH SURFACE, HORIZONTAL (X-AXIS) FIRST, VERTICAL (Y-AXIS) SECOND
    MatrixXd surface_lines(12,6);

    surface_lines <<//  GROUND AREA correct
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
    MatrixXd points(1,number*3);

    switch (number) {
        case 1:
            return (base_vector + (direction_vector*0.5)); // middle
        case 2:
            points << base_vector + 0.1* direction_vector, base_vector + 0.9*direction_vector; // left side right side
            return points;
        case 3:
            points << base_vector + (direction_vector*0.5), base_vector + 0.1* direction_vector, base_vector + 0.9*direction_vector; // middle, left side right side
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

MatrixXd merge_rotation(MatrixXd obj_rpy, MatrixXd joint_rpy){ // TODO NEEDS FIX
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

MatrixXd get_poses(MatrixXd &quaternions, MatrixXd &points){
    /*
     *
     */ // last 4 cols always quaternions len(row)-4 /3 = number of points in matrix row/calculated for this surface line
    MatrixXd pose_matrix(points.rows()*3, points.cols()+4);// 12 surface lines with 3 possible rots each 36 - rows: p1 p2 p3 q1 / p1.. q2 / p1.. q3/ then next point
    for(int i =0 ; i< 12; i++){
        int k = i*3;
        for(int j = 0; j<12 ; j+=4){
            pose_matrix.row(k) << points.row(i) , quaternions.block(floor(i/2),j,1,4);
            k+=1;
        }
    }
    /*
     *  POSE MATRIX , EACH POINT HAS 3 POSSIBLE QUATERNIONS, EACH SURFACE HAS 2 WAYS TO GET THE POINTS VERTICAL / HORIZONTAL
     *  POSE1 = SURFACE1_POINTS1_QUATERNION1
     *  POSE2 = SURFACE1_POINTS1_QUATERNION2
     *  POSE3 = SURFACE1_POINTS1_QUATERNION3
     *
     *  POSE4 = SURFACE1_POINTS2_QUATERNION1
     *  POSE5 = SURFACE1_POINTS2_QUATERNION2
     *  POSE6 = SURFACE1_POINTS2_QUATERNION3
     *
     *  POSE6 = SURFACE2_POINTS3_QUATERNION1
     *  POSE6 = SURFACE2_POINTS3_QUATERNION2
     *  POSE6 = SURFACE2_POINTS3_QUATERNION3
     */
    return pose_matrix;
}

MatrixXd sort_pose_matrix(MatrixXd pose_matrix){
    int points = (pose_matrix.cols()-4)/3; // 3-p1xyz 3-p2xyz 3-p3 xyz +quat 4
    MatrixXd new_matrix(pose_matrix.rows()*points,7);
    for(int i = 0; i<pose_matrix.rows(); i++) {
        int k = i * points;
        for (int j = 0; j < points * 3; j = j + 3) {
            new_matrix.row(k) << pose_matrix.block(i, j, 1, 3), pose_matrix.block(i, (points * 3), 1, 4);
            k += 1;
        }
    }
    return new_matrix;
}

MatrixXd identify_points_surface(int rows, int point_pos, MatrixXd &normals){
    // minimum matrix size 36*n - 7 (12 surface lines for 6 surfaces, 3 quaternions for each point on the same surface)
    int number_of_points = rows/36;
    // each surface needs
    int surface_points_it = number_of_points*6;
    int mod = point_pos / surface_points_it;

    return normals.row(mod);
}


std::vector<MatrixXd> get_pose_all_points(Vector3d &joint_pos, Vector3d &joint_rpy, Vector3d &object_size){

    std::vector<double> degr;
    degr.resize(joint_rpy.size());
    VectorXd::Map(&degr[0],joint_rpy.size()) = joint_rpy;
    MatrixXd vertices = get_rotated_object(degr, joint_pos,object_size);
    MatrixXd surface_equation = get_surface_equation(vertices);
    MatrixXd normals = get_normal_of_plane(surface_equation);

    std::cout << "normals " << std::endl;
    std::cout << normals  << std::endl;
    std::cout << "normals " << std::endl;
    MatrixXd points = generate_points_each_surface(surface_lines(get_middle_points(vertices)),1);

    MatrixXd joint_rpy_matrix(1,3);
    joint_rpy_matrix << joint_rpy[0,0],joint_rpy[0,1],joint_rpy[0,2];
    std::cout << joint_rpy_matrix << std::endl;

    MatrixXd quaternions = translate_rotations(joint_rpy_matrix); // calculate corresponding quaternions for pose

    MatrixXd pose_matrix = sort_pose_matrix(get_poses(quaternions,points));
    std::cout << "pose_matrix" << std::endl;
    std::cout << pose_matrix << std::endl;
    std::cout << "pose_matrix" << std::endl;

    std::random_device rd;     // only used once to initialise (seed) engine
    std::mt19937 rng(rd());    // random-number engine used (Mersenne-Twister in this case)
    std::uniform_int_distribution<int> uni(0,pose_matrix.rows()-1); // guaranteed unbiased
    int random_integer = uni(rng);

    MatrixXd normal_vec = identify_points_surface(pose_matrix.rows(),random_integer, normals);

    std::vector<MatrixXd> res;
    res.push_back(pose_matrix.row(random_integer));
    res.push_back(normal_vec);

    return res;
}

MatrixXd get_random_pose(Vector3d joint_pos,Vector3d joint_rpy,Vector3d object_size){
    std::vector<MatrixXd> pose_matrix_norm = get_pose_matrix(joint_pos , joint_rpy, object_size);
    MatrixXd pose_matrix = pose_matrix_norm.front();
    std::cout<< pose_matrix << std::endl;
   // std::cout << "my random int : " << random_integer << std::endl;

    MatrixXd pose_and_normal(1,10);
    pose_and_normal << pose_matrix, pose_matrix_norm[1] ;
    /*
    auto normassss = pose_matrix_norm[1];
    std::cout << normassss << std::endl;
    std::cout << "POSE AND NORMAALLLL" << std::endl;
    std::cout << pose_and_normal << std::endl;
    */
     return pose_and_normal;

}

std::vector<std::pair<std::string, MatrixXd>> get_pose_object (int object_no){
    read_srdf_file("cube_scene_srdf.txt");
    read_urdf_file("cube_scene.txt");
    Vector3d joint_pos(joint_positions[object_no].data());
    Vector3d joint_rpy(joint_rotations[object_no].data());

    Vector3d object_size(link_sizes[object_no].data());
    Vector3d object_rpy (link_rotations[object_no].data());
    Vector3d object_xyz (link_positions[object_no].data());
    std::string object_gr_name = group_names[0].data();

    MatrixXd random_pose = get_random_pose(joint_pos,joint_rpy,object_size);

    std::vector<std::pair<std::string,MatrixXd>> result;
    result.push_back(std::make_pair(object_gr_name,random_pose));
    return result;
}

std::vector<MatrixXd> get_pose_matrix(Vector3d joint_pos,Vector3d joint_rpy,Vector3d object_size) {


    std::vector<MatrixXd> pose_matrix =  get_pose_all_points(joint_pos,joint_rpy,object_size);

    /*
     * TODO ADJUST JOINT POS IF OBJECT XYZ NOT 0 0 0 // ROTATION ? JOINT ROT POS CORRECT MISSING OBJECT/LINK XYZ RPY
     */
    return pose_matrix;
}

void create_txt_from_urdf(){
    // RUN URDF2TXT

    Py_Initialize();
    FILE *fp;
    fp = _Py_fopen("/home/serboba/PycharmProjects/urdf_extract2txt/urdf2txt.py", "r+");
    PyRun_SimpleFile(fp, "/home/serboba/PycharmProjects/urdf_extract2txt/urdf2txt.py");
    Py_Finalize();

}