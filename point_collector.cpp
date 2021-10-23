//
// Created by serboba on 21.10.21.
//
//
// Created by serboba on 21.10.21.
//

#include <iostream>
#include <fstream>

#include <robowflex_library/util.h>
#include <Eigen/Dense>


using namespace robowflex;

Eigen::MatrixXd get_object_vertices(Eigen::Vector3d joint_pos, Eigen::Vector3d object_size){
    Eigen::MatrixXd vertices(8,3);
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


Eigen::MatrixXd get_surface_equation(Eigen::MatrixXd vertices){

    Eigen::MatrixXd surface_equation(6,9);
    surface_equation <<
    // grundflaeche
            vertices.row(0).x(), vertices.row(0).y(), vertices.row(0).z(), vertices.row(1)- vertices.row(0), vertices.row(2)- vertices.row(0),
            // seite links
            vertices.row(0).x(), vertices.row(0).y(), vertices.row(0).z(), vertices.row(4)- vertices.row(0), vertices.row(1)- vertices.row(0),
            //seite rechts
            vertices.row(2).x(), vertices.row(2).y(), vertices.row(2).z(), vertices.row(3)- vertices.row(2), vertices.row(6)- vertices.row(2),
            //seite hinten
            vertices.row(0).x(), vertices.row(0).y(), vertices.row(0).z(), vertices.row(2)- vertices.row(0), vertices.row(4)- vertices.row(0),
            //seite vorne
            vertices.row(1).x(), vertices.row(1).y(), vertices.row(1).z(), vertices.row(5)- vertices.row(1), vertices.row(3)- vertices.row(1),
            //seite oben
            vertices.row(4).x(), vertices.row(4).y(), vertices.row(4).z(), vertices.row(5)- vertices.row(4), vertices.row(6)- vertices.row(4);
    return surface_equation;
    //TODO CALCULATE NORM VEKTOR OF RV1 RV2 -> FIND AXIS TO ORIENTATE ADD IT TO THE MATRIX?
}

Eigen::MatrixXd surface_lines(Eigen::MatrixXd mid_point){
    // CALC LINES BETWEEN TWO MIDDLE POINTS - HORIZONTAL AND VERTICAL FOR EACH SURFACE, HORIZONTAL (X-AXIS) FIRST, VERTICAL (Y-AXIS) SECOND
    Eigen::MatrixXd surface_lines(12,6);

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

Eigen::MatrixXd get_middle_points(Eigen::MatrixXd vertices){
    Eigen::MatrixXd edge_middle(12,3);

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

Eigen::MatrixXd generate_points_from_line(Eigen::MatrixXd line, int number){

    Eigen::Vector3d point;
    Eigen::Map<Eigen::MatrixXd> base_vector (line.data(),1,3);
    Eigen::Map<Eigen::MatrixXd> direction_vector (line.data()+3,1,3);
    std::cout << base_vector << std::endl;
    std::cout << direction_vector << std::endl;
    Eigen::MatrixXd points(number,3);

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
            for (int i = 1; i< n; i++) { //better if number %2 = 1
                double t = double(i)/double(n);
                points.row(i-1) = base_vector + t*direction_vector;
            }
            return points;
    }
}

int main() {

    Eigen::Vector3d joint_pos;
    joint_pos << 0.8,0,0.7;
    Eigen::Vector3d object_size;
    object_size << 0.03,0.3,0.2;

  //  std::cout << object_size(1) << std::endl;
    auto vertices = get_object_vertices(joint_pos,object_size);
    auto surface_equation = get_surface_equation(vertices);


    auto middle_po = get_middle_points(vertices);
    auto lines = surface_lines(middle_po);

    Eigen::MatrixXd line(1,6);
    line << 0.8,-0.3,0.5,0,0.6,0;
    auto point = generate_points_from_line(line,4);
    std::cout << point << std::endl;
    return 0;
}
