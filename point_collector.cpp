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
#include <tf2/LinearMath/Quaternion.h>

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
Eigen::MatrixXd rpy_to_quaternion(double roll, double pitch, double yaw){
    tf2::Quaternion q_new;
    q_new.setRPY(roll,pitch,yaw);
    Eigen::MatrixXd quaternion_(1,4);
    quaternion_ << q_new.getW(), q_new.getX(), q_new.getY(), q_new.getZ();
    return quaternion_;
}

tf2::Quaternion eigen_to_tfquaternion(Eigen::MatrixXd quaternion){
    tf2::Quaternion q_;
    q_.setW(quaternion(0));
    q_.setX(quaternion(1));
    q_.setY(quaternion(2));
    q_.setZ(quaternion(3));
    return q_;
}
tf2::Quaternion eigen_to_tfquaternion(Eigen::Quaterniond quaternion){
    tf2::Quaternion q_;
    q_.setW(quaternion.w());
    q_.setX(quaternion.x());
    q_.setY(quaternion.y());
    q_.setZ(quaternion.z());
    return q_;
}

Eigen::Quaterniond matrix_to_quaternion(Eigen::MatrixXd matrix){
    Quaterniond q;
    q.w() = matrix(0);
    q.x() = matrix(1);
    q.y() = matrix(2);
    q.z() = matrix(3);
    return q;
}

Eigen::Quaterniond tf_to_eigen_quaternion(tf2::Quaternion quaternion){
    Eigen::Quaterniond q_;
    q_.w() = quaternion.getW();
    q_.x() = quaternion.getX();
    q_.y() = quaternion.getY();
    q_.z() = quaternion.getZ();
    return q_;
}

Eigen::MatrixXd tf_to_eigen_matrix_q(tf2::Quaternion quaternion){
    Eigen::MatrixXd q_;
    q_(0) = quaternion.getW();
    q_(1) = quaternion.getX();
    q_(2) = quaternion.getY();
    q_(2) = quaternion.getZ();
    return q_;
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


MatrixXd quaternion_to_euler(Quaterniond q){
    return q.toRotationMatrix().eulerAngles(2,1,0);
}

MatrixXd get_rotated_vertex(Vector3d obj_rpy, Vector3d point, Vector3d joint_pos ){

    MatrixXd q1 = rpy_to_quaternion(obj_rpy[0],obj_rpy[1],obj_rpy[2]);

    return (joint_pos + matrix_to_quaternion(q1)*(point-joint_pos));
    //return (joint_pos+ get_quaternion_from_euler(degrees[2],degrees[1],degrees[0])*(point-joint_pos));
}

MatrixXd get_rotated_object(Vector3d obj_rpy, Vector3d joint_pos, Vector3d object_size){ //rotate for all vertices
    MatrixXd vertices = get_object_vertices(joint_pos,object_size);
    for(int i = 0 ; i< vertices.rows(); i++){
        Vector3d row = vertices.row(i);
        MatrixXd new_v = get_rotated_vertex(obj_rpy, row, joint_pos);
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


MatrixXd get_quaternion_unten(MatrixXd rpy){
    std::cout << "MATRIX RPY " << rpy << std::endl;
    MatrixXd q_(1,4);
    q_ << rpy_to_quaternion(rpy(0),rpy(1)-M_PI*0.5,rpy(2));  // unten
                   std::cout <<"CALC QUATERNION " << q_ << std::endl;

            return q_;

}
MatrixXd get_quaternion_links(MatrixXd rpy){
    MatrixXd q_(1,4);
    q_ << rpy_to_quaternion(rpy(0),rpy(1),rpy(2)+M_PI*0.5);  // links
           return q_;
}
MatrixXd get_quaternion_hinten(MatrixXd rpy){
    MatrixXd q_(1,4);
    q_ << rpy_to_quaternion(rpy(0),rpy(1)-M_PI,rpy(2));      // hinten
    return q_;

}
MatrixXd get_quaternion_rechts(MatrixXd rpy){
    MatrixXd q_(1,4);
    q_ << rpy_to_quaternion(rpy(0),rpy(1),rpy(2)-M_PI*0.5);  // rechts
    return q_;
}
MatrixXd get_quaternion_vorne(MatrixXd rpy){
    MatrixXd q_(1,4);
    q_ << rpy_to_quaternion(rpy(0),rpy(1),rpy(2));           // vorne
    return q_;
}
MatrixXd get_quaternion_oben(MatrixXd rpy){
    MatrixXd q_(1,4);
    q_ << rpy_to_quaternion(rpy(0),rpy(1)+M_PI*0.5,rpy(2));
    return q_;
}

MatrixXd quaternion_x_y_z(MatrixXd rpy){
    MatrixXd quaternions(6,4);
     // x achse != 0, 0, 0

     if(rpy(0)!= 0.0 && abs(rpy(0)) < 1.57)
     {
         if(rpy(1) == 0.0 && rpy(2) ==0.0)
         { // all values zero except x + between -1.57 < x < 1.57 FAIL  (ROLL<-> YAW ? FIX)
             quaternions << get_quaternion_unten(rpy), get_quaternion_links(rpy), get_quaternion_hinten(rpy),get_quaternion_rechts(rpy), get_quaternion_vorne(rpy), get_quaternion_oben(rpy);
         }
     }
     else if(rpy(0)>= 1.57 && rpy(0) < 3.14)
     {
         if(rpy(1) != 0.0)
         {
             if(rpy(1) >= 1.57 && rpy(1) < 3.14)
             { // 1.57 1.57 0 CORRECT
                 rpy(1) = 0;
                 rpy(0) = 0;
                 std::cout << "BURADAAAA" << std::endl;
                 quaternions << get_quaternion_rechts(rpy),get_quaternion_vorne(rpy), get_quaternion_unten(rpy), get_quaternion_hinten(rpy),get_quaternion_oben(rpy),get_quaternion_links(rpy)  ;

             }
             else if(rpy(1) <= -1.57 && rpy(1) > -3.14)
             { // 1.57 -1.57 0 CORRECT
                 rpy(1) = 0;
                 rpy(0) = 0;
                 quaternions << get_quaternion_rechts(rpy),get_quaternion_hinten(rpy), get_quaternion_oben(rpy) ,get_quaternion_vorne(rpy),get_quaternion_unten(rpy),get_quaternion_links(rpy);

             }
         }
         else if(rpy(2) != 0.0)
         {
             if(rpy(2)>= 1.57 && rpy(2) <3.14)
             { // x != 0, y=0, z!=0 CORRECT
                 rpy(2) = 0;
                 rpy(0) = 0;
                 quaternions <<get_quaternion_vorne(rpy),get_quaternion_unten(rpy),get_quaternion_rechts(rpy), get_quaternion_oben(rpy), get_quaternion_links(rpy),get_quaternion_hinten(rpy) ;
             }
             else if(rpy(2) <= -1.57 && rpy(2) > -3.14)
             { // CORRECT
                 rpy(2) = 0;
                 rpy(0) = 0;
                 quaternions <<get_quaternion_hinten(rpy),get_quaternion_unten(rpy),get_quaternion_links(rpy), get_quaternion_oben(rpy), get_quaternion_rechts(rpy),get_quaternion_vorne(rpy) ;
             }
         }
         else
         { // 1.57 0 0 CORRECT
             quaternions << get_quaternion_rechts(rpy), get_quaternion_unten(rpy), get_quaternion_hinten(rpy), get_quaternion_oben(rpy), get_quaternion_vorne(rpy), get_quaternion_links(rpy);
         }
     }
     else if(rpy(0) <= -1.57 && rpy(0) > -3.14)
     {
         if(rpy(1)!= 0.0)
         {
             if(rpy(1) >= 1.57 && rpy(1) < 3.14)
             { // x!= 0, y!=0, z!=0 CORRECT
                 // rpy -= 1.57 ?
                 rpy(1) = 0;
                 rpy(0) = 0;
                 quaternions << get_quaternion_links(rpy),  get_quaternion_hinten(rpy),get_quaternion_unten(rpy), get_quaternion_vorne(rpy), get_quaternion_oben(rpy), get_quaternion_rechts(rpy);
             }
             else if(rpy(1) <= -1.57 && rpy(1) > -3.14)
             { // x!= 0, y!=0, z!=0 CORRECT
                 rpy(1) = 0;
                 rpy(0) = 0;
                 quaternions << get_quaternion_links(rpy), get_quaternion_vorne(rpy), get_quaternion_oben(rpy), get_quaternion_hinten(rpy), get_quaternion_unten(rpy), get_quaternion_rechts(rpy);
             }

         }else if(rpy(2) != 0.0)
         {
             if(rpy(2)>= 1.57 && rpy(2) <3.14)
             { // x != 0, y=0, z!=0 CORRECT
                 rpy(2) = 0;
                 rpy(0) = 0;
                 quaternions << get_quaternion_hinten(rpy),get_quaternion_oben(rpy), get_quaternion_rechts(rpy), get_quaternion_unten(rpy), get_quaternion_links(rpy),get_quaternion_vorne(rpy);
             }
             else if(rpy(2) <= -1.57 && rpy(2) > -3.14)
             { // CORRECT
                 rpy(2) = 0;
                 rpy(0) = 0;
                 quaternions << get_quaternion_vorne(rpy),get_quaternion_oben(rpy), get_quaternion_links(rpy), get_quaternion_unten(rpy), get_quaternion_rechts(rpy),get_quaternion_hinten(rpy);
             }
         }
         else // -1.57 0 0 CORRECT
         {
             quaternions << get_quaternion_links(rpy), get_quaternion_oben(rpy), get_quaternion_hinten(rpy),get_quaternion_unten(rpy), get_quaternion_vorne(rpy), get_quaternion_rechts(rpy);
         }
     }
     else{
         // ??
     }

     /*
         if(rpy(0)!= 0  && abs(rpy(0)) < 1.57 ){  // -1.57 < z < 1.57
            //unten, links, hinten, rechts, vorne, oben
            quaternions << get_quaternion_unten(rpy), get_quaternion_links(rpy), get_quaternion_hinten(rpy),get_quaternion_rechts(rpy), get_quaternion_vorne(rpy), get_quaternion_oben(rpy);
        }else if(rpy(0)>= 1.57 && rpy(0) < 3.14){
            //left, oben, hinten, unten, vorne, rechts
            if(rpy(1) ==0.0 && rpy(2) ==0.0){
                quaternions << get_quaternion_rechts(rpy), get_quaternion_unten(rpy), get_quaternion_hinten(rpy), get_quaternion_oben(rpy), get_quaternion_vorne(rpy), get_quaternion_links(rpy);
            }
            if(rpy(2)>= 1.57 && rpy(2) <3.14){ // x != 0, y=0, z!=0
                quaternions << get_quaternion_rechts(rpy),get_quaternion_hinten(rpy),get_quaternion_oben(rpy), get_quaternion_vorne(rpy), get_quaternion_unten(rpy), get_quaternion_links(rpy);
            }else if(rpy(2) <= -1.57 && rpy(2) > -3.14){
                quaternions << get_quaternion_rechts(rpy),get_quaternion_vorne(rpy), get_quaternion_unten(rpy),get_quaternion_hinten(rpy), get_quaternion_oben(rpy), get_quaternion_links(rpy);
            }

            if(rpy(1) >= 1.57 && rpy(1) < 3.14){ // x!= 0, y!=0, z!=0
                quaternions << get_quaternion_hinten(rpy),get_quaternion_unten(rpy),get_quaternion_links(rpy), get_quaternion_oben(rpy), get_quaternion_rechts(rpy),get_quaternion_vorne(rpy);
                // rpy -= 1.57 ?
            }else if(rpy(1) <= -1.57 && rpy(1) > -3.14){ // x!= 0, y!=0, z!=0
                quaternions << get_quaternion_vorne(rpy), get_quaternion_unten(rpy), get_quaternion_rechts(rpy),get_quaternion_oben(rpy), get_quaternion_links(rpy), get_quaternion_hinten(rpy);
            }

            }else if(rpy(0) <= -1.57 && rpy(0) > -3.14){
                            // rechts, unten, hinten, oben, vorne, left
                            quaternions << get_quaternion_links(rpy), get_quaternion_oben(rpy), get_quaternion_hinten(rpy),get_quaternion_unten(rpy), get_quaternion_vorne(rpy), get_quaternion_rechts(rpy);

                            if(rpy(2)>= 1.57 && rpy(2) <3.14){ // x != 0, y=0, z!=0
                                quaternions << get_quaternion_links(rpy),get_quaternion_hinten(rpy), get_quaternion_unten(rpy), get_quaternion_vorne(rpy), get_quaternion_oben(rpy),get_quaternion_rechts(rpy);
                            }else if(rpy(2) <= -1.57 && rpy(2) > -3.14){
                                quaternions << get_quaternion_links(rpy), get_quaternion_vorne(rpy), get_quaternion_oben(rpy), get_quaternion_hinten(rpy), get_quaternion_unten(rpy), get_quaternion_rechts(rpy);
                            }

                            if(rpy(1) >= 1.57 && rpy(1) < 3.14){ // x!= 0, y!=0, z!=0
                                // rpy -= 1.57 ?
                                quaternions << get_quaternion_hinten(rpy), get_quaternion_oben(rpy), get_quaternion_rechts(rpy), get_quaternion_unten(rpy), get_quaternion_links(rpy), get_quaternion_vorne(rpy);
                            }else if(rpy(1) <= -1.57 && rpy(1) > -3.14){ // x!= 0, y!=0, z!=0
                                quaternions << get_quaternion_vorne(rpy), get_quaternion_oben(rpy), get_quaternion_links(rpy), get_quaternion_unten(rpy), get_quaternion_rechts(rpy), get_quaternion_hinten(rpy);
                            }

        }else{
            // ERROR CHECK CASES? + 3.14
        }
*/
    return quaternions;

}

MatrixXd quaternion_y_z(MatrixXd rpy)
{
    MatrixXd quaternions(6,4);
    if(rpy(1)!= 0.0 && abs(rpy(1)) < 1.57)
    {
        if(rpy(2) ==0.0)
        { // all values zero except y NEEDS FIX
            quaternions << get_quaternion_unten(rpy), get_quaternion_links(rpy), get_quaternion_hinten(rpy), get_quaternion_rechts(rpy), get_quaternion_vorne(rpy), get_quaternion_oben(rpy);
        }
    }
    else if(rpy(1)>= 1.57 && rpy(1) < 3.14)
    {
        //hinten, left, oben, rechts, unten, vorne

        if(rpy(2)!= 0.0)
        {
            if(rpy(2)>= 1.57 && rpy(2) <3.14)
            { // x != 0, y=0, z!=0 CORRECT
                rpy(1) =0;
                rpy(2) =0;
                quaternions << get_quaternion_links(rpy), get_quaternion_hinten(rpy), get_quaternion_unten(rpy),get_quaternion_vorne(rpy), get_quaternion_oben(rpy), get_quaternion_rechts(rpy);

            }
            else if(rpy(2) <= -1.57 && rpy(2) > -3.14)
            { //   CORRECT
                rpy(1) =0;
                rpy(2) =0;
                quaternions << get_quaternion_rechts(rpy), get_quaternion_vorne(rpy), get_quaternion_unten(rpy),get_quaternion_hinten(rpy), get_quaternion_oben(rpy), get_quaternion_links(rpy);
            }
        }else{ // CORRECT

            rpy(1) = 0;
            quaternions << get_quaternion_vorne(rpy), get_quaternion_links(rpy), get_quaternion_unten(rpy), get_quaternion_rechts(rpy), get_quaternion_oben(rpy), get_quaternion_hinten(rpy);
        }
    }
    else if(rpy(1) <= -1.57 && rpy(1) > -3.14)
    {
        // vorne, left, unten, rechts, oben, hinten
        if(rpy(2) != 0.0)
        {
            if(rpy(2)>= 1.57 && rpy(2) <3.14)
            { // x != 0, y=0, z!=0 CORRECT

                rpy(1) =0;
                rpy(2) =0;
                quaternions << get_quaternion_rechts(rpy), get_quaternion_hinten(rpy), get_quaternion_oben(rpy), get_quaternion_vorne(rpy), get_quaternion_unten(rpy), get_quaternion_links(rpy);
            }
            else if(rpy(2) <= -1.57 && rpy(2) > -3.14)
            {

                rpy(1) =0;
                rpy(2) =0;
                quaternions << get_quaternion_links(rpy), get_quaternion_vorne(rpy), get_quaternion_oben(rpy), get_quaternion_hinten(rpy), get_quaternion_unten(rpy), get_quaternion_rechts(rpy);
            }
        }
        else
        {
            rpy(1) = 0;
            quaternions <<  get_quaternion_hinten(rpy),get_quaternion_links(rpy),get_quaternion_oben(rpy) , get_quaternion_rechts(rpy) ,get_quaternion_unten(rpy) ,get_quaternion_vorne(rpy);
        }
    }
    else{
        // ?
    }
    return quaternions;

}

MatrixXd quaternion_z(MatrixXd rpy)
{ //CORRECT
    MatrixXd quaternions(6,4);
    quaternions<< get_quaternion_unten(rpy),get_quaternion_links(rpy),get_quaternion_hinten(rpy), get_quaternion_rechts(rpy), get_quaternion_vorne(rpy), get_quaternion_oben(rpy);
    return quaternions;

}

MatrixXd sort_quaternion(MatrixXd rpy){ // ROLL =
    MatrixXd quaternions(6,4);
    if (rpy(0) == 0.0 && rpy(1) == 0.0 && rpy(2) == 0.0){
        quaternions<< get_quaternion_unten(rpy),get_quaternion_links(rpy),get_quaternion_hinten(rpy), get_quaternion_rechts(rpy), get_quaternion_vorne(rpy), get_quaternion_oben(rpy);
    } // no rot


    if(rpy(0) != 0.0){
      quaternions <<  quaternion_x_y_z(rpy);
    }

    if(rpy(0) ==0.0 && rpy(1) != 0.0){
        quaternions << quaternion_y_z(rpy);
    }
    if(rpy(0) == 0.0 && rpy(1) == 0.0 && rpy(2) != 0.0){
        quaternions << quaternion_z(rpy);
    }

/*
    if (rpy(0) == 0.0 && rpy(1) == 0.0 && rpy(2) != 0.0){ // 0, 0, z achse != 0.0
        //if(abs(rpy(2))>=0 && abs(rpy(2)) < 1.57){  // -1.57 < z < 1.57
            //unten, links, hinten, rechts, vorne, oben
            quaternions<< get_quaternion_unten(rpy),get_quaternion_links(rpy),get_quaternion_hinten(rpy), get_quaternion_rechts(rpy), get_quaternion_vorne(rpy), get_quaternion_oben(rpy);


        }else if(rpy(2)>= 1.57 && rpy(2) < 3.14){
            //unten, vorne, left, hinten, rechts, oben
            quaternions << get_quaternion_unten(rpy),get_quaternion_hinten(rpy), get_quaternion_rechts(rpy), get_quaternion_vorne(rpy), get_quaternion_links(rpy), get_quaternion_oben(rpy);

            }else if(rpy(2) <= -1.57 && rpy(2) > -3.14){
            // unten , hinten, rechts, vorne, links, oben
            quaternions << get_quaternion_unten(rpy), get_quaternion_vorne(rpy), get_quaternion_links(rpy), get_quaternion_hinten(rpy), get_quaternion_rechts(rpy), get_quaternion_oben(rpy);

        }else{
            // ERROR CHECK CASES? + 3.14 ??
        }

    }

    if (rpy(0) == 0.0 && rpy(1) != 0.0 && rpy(2) == 0.0){ // 0, y achse != 0.0, 0 // todo z axis !?
        if(abs(rpy(1))>=0 && abs(rpy(1)) < 1.57){  // -1.57 < z < 1.57
            //unten, links, hinten, rechts, vorne, oben
            quaternions << get_quaternion_unten(rpy), get_quaternion_links(rpy), get_quaternion_hinten(rpy), get_quaternion_rechts(rpy), get_quaternion_vorne(rpy), get_quaternion_oben(rpy);
        }else if(rpy(1)>= 1.57 && rpy(1) < 3.14){
            //hinten, left, oben, rechts, unten, vorne
            rpy(1) -=1.57;
            quaternions << get_quaternion_vorne(rpy), get_quaternion_links(rpy), get_quaternion_unten(rpy), get_quaternion_rechts(rpy), get_quaternion_oben(rpy), get_quaternion_hinten(rpy);

                if(rpy(2)>= 1.57 && rpy(2) <3.14){ // x != 0, y=0, z!=0
                    quaternions << get_quaternion_vorne(rpy), get_quaternion_unten(rpy), get_quaternion_rechts(rpy),get_quaternion_oben(rpy), get_quaternion_links(rpy), get_quaternion_hinten(rpy);

                }else if(rpy(2) <= -1.57 && rpy(2) > -3.14){
                    quaternions << get_quaternion_vorne(rpy), get_quaternion_oben(rpy), get_quaternion_links(rpy), get_quaternion_unten(rpy), get_quaternion_rechts(rpy), get_quaternion_hinten(rpy);
                }

        }else if(rpy(1) <= -1.57 && rpy(1) > -3.14){
            // vorne, left, unten, rechts, oben, hinten
            rpy(1) -=1.57;
            quaternions <<  get_quaternion_hinten(rpy), get_quaternion_links(rpy),get_quaternion_oben(rpy) , get_quaternion_rechts(rpy), get_quaternion_unten(rpy),get_quaternion_vorne(rpy);

                if(rpy(2)>= 1.57 && rpy(2) <3.14){ // x != 0, y=0, z!=0
                    quaternions << get_quaternion_hinten(rpy), get_quaternion_oben(rpy), get_quaternion_rechts(rpy), get_quaternion_unten(rpy), get_quaternion_links(rpy), get_quaternion_vorne(rpy);

                }else if(rpy(2) <= -1.57 && rpy(2) > -3.14){
                    quaternions << get_quaternion_hinten(rpy), get_quaternion_unten(rpy), get_quaternion_links(rpy), get_quaternion_oben(rpy), get_quaternion_rechts(rpy), get_quaternion_vorne(rpy);

                }

        }else{
            // ERROR CHECK CASES? + 3.14
        }
    }
*/

    // TODO CHECK DIFFERENT ROTATIONS + COMBINATIONS OF THEM (F.E. 0,1.57,1.57) AND SO ON, -> HARDCODE AS ABOVE BUT, FIX 3.14 AND ELSE CASE BEFORE THAT
    std::cout << quaternions << std::endl;
    return quaternions;

}


MatrixXd translate_rotations(MatrixXd rpy){

     return sort_quaternion(rpy);
}

MatrixXd new_rotation_quaternion(MatrixXd rpy,Eigen::Quaterniond q_original, int surface_no){ // rpy sum = actual rot + wanted rot
    tf2::Quaternion q_rot;
    switch (surface_no) {
        case 0: case 1: case 2: case 15: case 16: case 17:          // unten
            q_rot = eigen_to_tfquaternion(rpy_to_quaternion(rpy(0),rpy(1),rpy(2)));             //unten
            break;
        case 3: case 4: case 5:  case 6: case 7: case 8: case 9: case 10: case 11: case 12: case 13: case 14:
            q_rot = eigen_to_tfquaternion(rpy_to_quaternion(rpy(0),rpy(1),rpy(2)));          // oben
            break;
    }
    tf2::Quaternion q_org = eigen_to_tfquaternion(q_original);
    tf2::Quaternion q_new;
    q_new = q_rot*q_org;
    q_new.normalize();
    return tf_to_eigen_matrix_q(q_new);

}


Eigen::Quaterniond rotate_quaternion(double roll, double pitch, double yaw, Quaterniond old_q){
    tf2::Quaternion q_rot;
    q_rot.setRPY(roll,pitch,yaw);
    tf2::Quaternion q_old;
    q_old.setW(old_q.w());
    q_old.setX(old_q.x());
    q_old.setY(old_q.y());
    q_old.setZ(old_q.z());
    tf2::Quaternion q_new;
    q_new = q_rot*q_old;
    q_new.normalize();
    Eigen::Quaterniond q_res;
    q_res.w() = q_new.getW();
    q_res.x() = q_new.getX();
    q_res.y() = q_new.getY();
    q_res.z() = q_new.getZ();
    return q_res;
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
    read_srdf_file("grasp_srdf.txt");
    read_urdf_file("grasp.txt");
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
    std::cout << "YO ? <" << quaternions << std::endl;
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
        std::cout <<"RANDOM RATIO VALUE : "<< ratio_vec << std::endl;
    }
    return ratio;
}

MatrixXd get_point_position(MatrixXd surface_equation,std::vector<double> ratio_dir_vectors){

    Map<MatrixXd> position_vector(surface_equation.data(),1,3);
    Map<MatrixXd> direction_vector1(surface_equation.data()+3,1,3);
    Map<MatrixXd> direction_vector2(surface_equation.data()+6,1,3);
    direction_vector1 *= ratio_dir_vectors[0];
    direction_vector2 *= ratio_dir_vectors[1];
    std::cout <<  "RATIO DIR VECTOR 1 " << ratio_dir_vectors[0] << std::endl;
    std::cout <<  "RATIO DIR VECTOR 2 " << ratio_dir_vectors[1] << std::endl;

    return (position_vector+direction_vector1+direction_vector2);
}

MatrixXd get_random_point_from_surface(MatrixXd surface_equation_matrix, MatrixXd rpy_matrix, MatrixXd normals){ // eine ebenengleichung
    srand ( time(NULL) );
    int surface_no =rand() %6; // SURFACE NUMBER-> NEED IT FOR QUATERNIONS !!! DONT LOSE IT
    int quaternion_no = 0;
    int surf_q_no = (surface_no*3)+quaternion_no; // surf no w quats
    std::cout << " SURFACE NO: " << surface_no << std::endl;
    std::cout << " QUAT NO: " << quaternion_no << std::endl;

    MatrixXd surface_equation = extract_surface(surface_no, surface_equation_matrix);
    MatrixXd quaternion = match_quaternion_to_surface(quaternion_no,surface_no,rpy_matrix);

    MatrixXd position(1,11);
    position << get_point_position(surface_equation,get_ratio_dir_vecs()),quaternion,normals.row(surface_no), surf_q_no;
    return position;

}

Eigen::MatrixXd get_pose_object(Object &obj){


    obj.actual_position = get_rotated_vertex(obj.actual_rotation, (obj.link_xyz + obj.joint_xyz), obj.joint_xyz); // ADJUST JOINT POS W RESPECT TO LINK XYZ

    std::cout <<  "Actual position : " << obj.actual_position << std::endl;
    std::cout <<  "Actual rotation : " << obj.actual_rotation << std::endl;
    auto vertices = get_rotated_object(obj.actual_rotation, obj.actual_position,obj.link_size);
    MatrixXd surface_equation = get_surface_equation(get_rotated_object(obj.actual_rotation, obj.actual_position,obj.link_size));
    MatrixXd normals = get_normal_of_plane(surface_equation);
    MatrixXd rpy_matrix= vec_to_matrix(obj.link_rpy+obj.joint_rpy);

    MatrixXd random_pose = get_random_point_from_surface(surface_equation,rpy_matrix,normals);

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
