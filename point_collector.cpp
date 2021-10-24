//
// Created by serboba on 21.10.21.
//
//
// Created by serboba on 21.10.21.
//

#include <iostream>
#include <fstream>

#include <Eigen/Dense>
#include <robowflex_library/util.h>
using namespace Eigen;

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


MatrixXd get_rotated_vertex(std::vector<double> degrees, Vector3d point, Vector3d joint_pos ){
    return (joint_pos+(robowflex::Rotation_Helper::calculateCS(degrees))*(point-joint_pos));
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

Quaternionf get_quaternion_from_euler(float yaw, float pitch, float roll){
    Quaternionf q;
    q =  AngleAxisf (yaw,Vector3f::UnitZ())*AngleAxisf (pitch,Vector3f::UnitY()) *AngleAxisf (roll,Vector3f::UnitX());
    return q;
}

MatrixXf quaternion_to_euler(Quaternionf q){
    return q.toRotationMatrix().eulerAngles(2,1,0);
}

int main() {

    Vector3d joint_pos;
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
   //std::cout << get_rotobj << std::endl;

   auto s = get_quaternion_from_euler(-1.57,0,1.57);

    std::cout << "Quaternionff: "  << s.coeffs() << std::endl;
    std::cout << s.w() <<" , " << s.x() <<" , "<< s.y() << " , " << s.z() << std::endl;

   auto d = quaternion_to_euler(s);

   std::cout << "euler : " << d << std::endl;
    return 0;
}
