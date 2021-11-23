//
// Created by serboba on 21.11.21.
//

#include <robowflex_dart/quaternion_factory.h>

//TODO instead of rpy(x) = 0, use -= or += 1.57

MatrixXd rpy_to_quaternion(double roll, double pitch, double yaw){
    tf2::Quaternion q_new;
    q_new.setRPY(roll,pitch,yaw);
    Eigen::MatrixXd quaternion_(1,4);
    quaternion_ << q_new.getW(), q_new.getX(), q_new.getY(), q_new.getZ();
    return quaternion_;
}

MatrixXd get_quaternion_unten(MatrixXd rpy){
    MatrixXd q_(1,4);
    q_ << rpy_to_quaternion(rpy(0),rpy(1)-M_PI*0.5,rpy(2));  // unten
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

MatrixXd match_deg_to_rpy(MatrixXd rpy, MatrixXd axis){ // todo +add wanted rotation not only 90 deg
    MatrixXd degree(1,3);
    if(rpy(0) == 0 && rpy(1) == 0 && rpy(2) == 0){
        if(axis(0) == 1)
            degree << 1.57, 0.0, 0.0;
        if(axis(1) == 1)
            degree << 0.0, 1.57, 0.0;
        if(axis(2) == 1)
            degree << 0.0, 0.0, 1.57;
        if(axis(0) == -1)
            degree << -1.57, 0.0, 0.0;
        if(axis(1) == -1)
            degree << 0.0, -1.57, 0.0;
        if(axis(2) == -1)
            degree << 0.0, 0.0, -1.57;
    }
    if(rpy(0) == 1.57 && rpy(1) == 0 && rpy(2) == 0){
        if(axis(0) == 1)
            degree << 1.57, 0.0, 0.0;
        if(axis(1) == 1)
            degree << 0.0, 0.0, 1.57;
        if(axis(2) == 1)
            degree << 0.0, -1.57,0.0;
        if(axis(0) == -1)
            degree << -1.57, 0.0, 0.0;
        if(axis(1) == -1)
            degree << 0.0, 0.0, -1.57;
        if(axis(2) == -1)
            degree << 0.0, 1.57, 0.0;
    }
    if(rpy(0) == -1.57 && rpy(1) == 0 && rpy(2) == 0){
       if(axis(0) == 1)
            degree << 1.57, 0.0, 0.0;
        if(axis(1) == 1)
            degree << 0.0, 0.0, -1.57;
        if(axis(2) == 1)
            degree << 0.0, 1.57,0.0;
        if(axis(0) == -1)
            degree << -1.57, 0.0, 0.0;
        if(axis(1) == -1)
            degree << 0.0, 0.0, 1.57;
        if(axis(2) == -1)
            degree << 0.0, -1.57, 0.0;
    }
    if(rpy(0) == 0 && rpy(1) == 1.57 && rpy(2) == 0){
        if(axis(0) == 1)
            degree << 0.0,0.0,-1.57;
        if(axis(1) == 1)
            degree << 0.0,1.57,0.0;
        if(axis(2) == 1)
            degree << 1.57,0.0,0.0;
        if(axis(0) == -1)
            degree << 0.0,0.0,1.57;
        if(axis(1) == -1)
            degree << 0.0,-1.57,0.0;
        if(axis(2) == -1)
            degree << -1.57,0.0,0.0;
    }
    if(rpy(0) == 0 && rpy(1) == -1.57 && rpy(2) == 0){
        if(axis(0) == 1)
            degree << 0.0,0.0,1.57;
        if(axis(1) == 1)
            degree << 0.0,1.57,0.0;
        if(axis(2) == 1)
            degree << -1.57,0.0,0.0;
        if(axis(0) == -1)
            degree << 0.0,0.0,-1.57;
        if(axis(1) == -1)
            degree << 0.0,-1.57,0.0;
        if(axis(2) == -1)
            degree << 1.57,0.0,0.0;
    }
    if(rpy(0) == 0 && rpy(1) == 0 && rpy(2) == 1.57){
        if(axis(0) == 1)
            degree << 0.0,1.57,0.0;
        if(axis(1) == 1)
            degree << -1.57,0.0,0.0;
        if(axis(2) == 1)
            degree << 0.0,0.0,1.57;
        if(axis(0) == -1)
            degree << 0.0,-1.57,0.0;
        if(axis(1) == -1)
            degree << 1.57,0.0,0.0;
        if(axis(2) == -1)
            degree << 0.0,0.0,-1.57;

    }
    if(rpy(0) == 0 && rpy(1) == 0 && rpy(2) == -1.57){
        if(axis(0) == 1)
            degree << 0.0,-1.57,0.0;
        if(axis(1) == 1)
            degree << 1.57,0.0,0.0;
        if(axis(2) == 1)
            degree << 0.0,0.0,1.57;
        if(axis(0) == -1)
            degree << 0.0,1.57,0.0;
        if(axis(1) == -1)
            degree << -1.57,0.0,0.0;
        if(axis(2) == -1)
            degree << 0.0,0.0,-1.57;
    }
    if(rpy(0) == 1.57 && rpy(1) == 1.57 && rpy(2) == 0){
        if(axis(0) == 1)
            degree << 0.0,0.0,-1.57;
        if(axis(1) == 1)
            degree << 1.57,0.0,0.0;
        if(axis(2) == 1)
            degree << 0.0,-1.57,0.0;
        if(axis(0) == -1)
            degree << 0.0,0.0,1.57;
        if(axis(1) == -1)
            degree << -1.57,0.0,0.0;
        if(axis(2) == -1)
            degree << 0.0,1.57,0.0;
    }
    if(rpy(0) == 1.57 && rpy(1) == -1.57 && rpy(2) == 0){
        if(axis(0) == 1)
            degree << 0.0,0.0,1.57;
        if(axis(1) == 1)
            degree << -1.57,0.0,0.0;
        if(axis(2) == 1)
            degree << 0.0,-1.57,0.0;
        if(axis(0) == -1)
            degree << 0.0,0.0,-1.57;
        if(axis(1) == -1)
            degree << 1.57,0.0,0.0;
        if(axis(2) == -1)
            degree << 0.0,1.57,0.0;
    }
    if(rpy(0) == -1.57 && rpy(1) == 1.57 && rpy(2) == 0){
        if(axis(0) == 1)
            degree << 0.0,0.0,-1.57;
        if(axis(1) == 1)
            degree << -1.57,0.0,0.0;
        if(axis(2) == 1)
            degree << 0.0,1.57,0.0;
        if(axis(0) == -1)
            degree << 0.0,0.0,1.57;
        if(axis(1) == -1)
            degree << 1.57,0.0,0.0;
        if(axis(2) == -1)
            degree << 0.0,1.57,0.0;
    }
    if(rpy(0) == -1.57 && rpy(1) == -1.57 && rpy(2) == 0){
        if(axis(0) == 1)
            degree << 0.0,0.0,1.57;
        if(axis(1) == 1)
            degree << 1.57,0.0,0.0;
        if(axis(2) == 1)
            degree << 0.0,1.57,0.0;
        if(axis(0) == -1)
            degree << 0.0,0.0,-1.57;
        if(axis(1) == -1)
            degree << -1.57,0.0,0.0;
        if(axis(2) == -1)
            degree << 0.0,-1.57,0.0;
    }
    if(rpy(0) == 1.57 && rpy(1) == 0 && rpy(2) == 1.57){
        if(axis(0) == 1)
            degree << 0.0,1.57,0.0;
        if(axis(1) == 1)
            degree << 0.0,0.0,1.57;
        if(axis(2) == 1)
            degree << 1.57,0.0,0.0;
        if(axis(0) == -1)
            degree << 0.0,-1.57,0.0;
        if(axis(1) == -1)
            degree << 0.0,0.0,-1.57;
        if(axis(2) == -1)
            degree << -1.57,0.0,0.0;
    }
    if(rpy(0) == 1.57 && rpy(1) == 0 && rpy(2) == -1.57){
        if(axis(0) == 1)
            degree << 0.0,-1.57,0.0;
        if(axis(1) == 1)
            degree << 0.0,0.0,1.57;
        if(axis(2) == 1)
            degree << -1.57,0.0,0.0;
        if(axis(0) == -1)
            degree << 0.0,1.57,0.0;
        if(axis(1) == -1)
            degree << 0.0,0.0,-1.57;
        if(axis(2) == -1)
            degree << 1.57,0.0,0.0;
    }
    if(rpy(0) == -1.57 && rpy(1) == 0 && rpy(2) == 1.57){
        if(axis(0) == 1)
            degree << 0.0,1.57,0.0;
        if(axis(1) == 1)
            degree << 0.0,0.0,-1.57;
        if(axis(2) == 1)
            degree << -1.57,0.0,0.0;
        if(axis(0) == -1)
            degree << 0.0,-1.57,0.0;
        if(axis(1) == -1)
            degree << 0.0,0.0,1.57;
        if(axis(2) == -1)
            degree << 1.57,0.0,0.0;
    }
    if(rpy(0) == -1.57 && rpy(1) == 0 && rpy(2) == -1.57){
        if(axis(0) == 1)
            degree << 0.0,-1.57,0.0;
        if(axis(1) == 1)
            degree << 0.0,0.0,-1.57;
        if(axis(2) == 1)
            degree << 1.57,0.0,0.0;
        if(axis(0) == -1)
            degree << 0.0,1.57,0.0;
        if(axis(1) == -1)
            degree << 0.0,0.0,1.57;
        if(axis(2) == -1)
            degree << -1.57,0.0,0.0;
    }
    if(rpy(0) == 0 && rpy(1) == 1.57 && rpy(2) == 1.57){
        if(axis(0) == 1)
            degree << 0.0,0.0,-1.57;
        if(axis(1) == 1)
            degree << -1.57,0.0,0.0;
        if(axis(2) == 1)
            degree << 0.0,1.57,0.0;
        if(axis(0) == -1)
            degree << 0.0,0.0,1.57;
        if(axis(1) == -1)
            degree << 1.57,0.0,0.0;
        if(axis(2) == -1)
            degree << 0.0,1.57,0.0;
    }
    if(rpy(0) == 0 && rpy(1) == 1.57 && rpy(2) == -1.57){
        if(axis(0) == 1)
            degree << 0.0,0.0,-1.57;
        if(axis(1) == 1)
            degree << 1.57,0.0,0.0;
        if(axis(2) == 1)
            degree << 0.0,-1.57,0.0;
        if(axis(0) == -1)
            degree << 0.0,0.0,1.57;
        if(axis(1) == -1)
            degree << -1.57,0.0,0.0;
        if(axis(2) == -1)
            degree << 0.0,1.57,0.0;
    }
    if(rpy(0) == 0 && rpy(1) == -1.57 && rpy(2) == 1.57){
        if(axis(0) == 1)
            degree << 0.0,0.0,1.57;
        if(axis(1) == 1)
            degree << -1.57,0.0,0.0;
        if(axis(2) == 1)
            degree << 0.0,-1.57,0.0;
        if(axis(0) == -1)
            degree << 0.0,0.0,-1.57;
        if(axis(1) == -1)
            degree << 1.57,0.0,0.0;
        if(axis(2) == -1)
            degree << 0.0,1.57,0.0;
    }
    if(rpy(0) == 0 && rpy(1) == -1.57 && rpy(2) == -1.57){
        if(axis(0) == 1)
            degree << 0.0,0.0,1.57;
        if(axis(1) == 1)
            degree << 1.57,0.0,0.0;
        if(axis(2) == 1)
            degree << 0.0,1.57,0.0;
        if(axis(0) == -1)
            degree << 0.0,0.0,-1.57;
        if(axis(1) == -1)
            degree << -1.57,0.0,0.0;
        if(axis(2) == -1)
            degree << 0.0,-1.57,0.0;
    }
    if(rpy(0) == 0 && rpy(1) == 0 && rpy(2) == 1.57){
        if(axis(0) == 1)
                degree << 0.0,1.57,0.0;
        if(axis(1) == 1)
            degree << -1.57,0.0,0.0;
        if(axis(2) == 1)
            degree << 0.0,0.0,1.57;
        if(axis(0) == -1)
            degree << 0.0,-1.57,0.0;
        if(axis(1) == -1)
            degree << 1.57,0.0,0.0;
        if(axis(2) == -1)
            degree << 0.0,0.0,-1.57;
    }
    if(rpy(0) == 0 && rpy(1) == 0 && rpy(2) == -1.57){
        if(axis(0) == 1)
            degree << 0.0,-1.57,0.0;
        if(axis(1) == 1)
            degree << 1.57,0.0,0.0;
        if(axis(2) == 1)
            degree << 0.0,0.0,1.57;
        if(axis(0) == -1)
            degree << 0.0,1.57,0.0;
        if(axis(1) == -1)
            degree << 1.57,0.0,0.0;
        if(axis(2) == -1)
            degree << 0.0,0.0,-1.57;
    }
    return degree;

}


