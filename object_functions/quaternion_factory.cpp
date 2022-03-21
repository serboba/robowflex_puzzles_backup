//
// Created by serboba on 21.11.21.
//

#include <robowflex_dart/quaternion_factory.h>


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

MatrixXd rmv(MatrixXd rpy, MatrixXd quaternion){ // r = rotate m = missing v = value

        tf2::Quaternion q_rot;
        q_rot = eigen_to_tfquaternion(rpy_to_quaternion(rpy(0),rpy(1),rpy(2)));             //unten

        tf2::Quaternion q_org = eigen_to_tfquaternion(quaternion);
        tf2::Quaternion q_new;
        q_new = q_rot*q_org;
         q_new.normalize();

         tf_quaternion_to_rpy(q_new );  //print
        return tf_to_eigen_matrix_q(q_new);
}

// todo recursive? or better function maybe
std::vector<MatrixXd> divide_rpy(int axis, MatrixXd rpy){
    std::vector<MatrixXd> new_rpys;

    MatrixXd temp_rpy(1,3);
    temp_rpy.setZero();
    if(abs(rpy(axis)) < 1.57){ // 0.90 1248 238 andere egal wir extrahieren nur den ersten wert
        temp_rpy(axis) = rpy(axis);
        new_rpys.push_back(temp_rpy);
    }else if(abs(rpy(axis)) == 1.57){
        temp_rpy(axis) = rpy(axis);
        new_rpys.push_back(temp_rpy);
    }else if(rpy(axis) > 1.57 && rpy(axis) < 3.14){ // 1.90
        temp_rpy(axis) = 1.57;
        new_rpys.push_back(temp_rpy);
        temp_rpy.setZero();
        temp_rpy(axis) = rpy(axis) - 1.57;
        new_rpys.push_back(temp_rpy);
    }else if(rpy(axis) < -1.57 && rpy(axis) > -3.14){
        temp_rpy(axis) = -1.57;
        new_rpys.push_back(temp_rpy);
        temp_rpy.setZero();
        temp_rpy(axis) = rpy(axis) + 1.57;
        new_rpys.push_back(temp_rpy);
    }else if(rpy(axis) > 3.14){
        temp_rpy(axis) = 1.57;
        new_rpys.push_back(temp_rpy);
        new_rpys.push_back(temp_rpy);
        temp_rpy(axis) = rpy(axis) - 3.14;
        new_rpys.push_back(temp_rpy);
        // todo 3.14 and over
    }else if(rpy(axis) < -3.14){
        temp_rpy(axis) = -1.57;
        new_rpys.push_back(temp_rpy);
        new_rpys.push_back(temp_rpy);
        temp_rpy(axis) = rpy(axis) + 3.14;
        new_rpys.push_back(temp_rpy);
        // todo 3.14 and over
    }else{

    }
    return new_rpys;

}

std::vector<MatrixXd> rpy_to_vector(MatrixXd rpy){

    std::vector<MatrixXd> rpys;
    for(int i = 0; i < 3 ; i++){
        if(rpy(i) != 0.0){
            std::vector<MatrixXd> r_ = divide_rpy(i,rpy);
            rpys.insert(rpys.end(), r_.begin(), r_.end());
        }
    }
    return rpys;
}

MatrixXd actual_quaternion(MatrixXd obj_rpy){
    MatrixXd quaternion(1,4);
    std::vector<MatrixXd> obj_rpys = rpy_to_vector(obj_rpy);
    MatrixXd rp_new(1,3);
    rp_new << 0.0, 0.0, 0.0;
    quaternion << rpy_to_quaternion(rp_new(0),rp_new(1),rp_new(2));
    for(size_t i= 0; i < obj_rpys.size() ; i++){
            quaternion = rmv(obj_rpys[i],quaternion);
    }
    return quaternion;
}

MatrixXd actual_quaternion(MatrixXd rot_rpy, int surf_no){
    MatrixXd quaternion(1,4);
    std::vector<MatrixXd> rot_rpys = rpy_to_vector(rot_rpy);
    MatrixXd rp_new(1,3);
    rp_new << 0.0, 0.0, 0.0;
    switch (surf_no) {
        case 0:
            quaternion<< get_quaternion_unten(rp_new);
            break;
        case 1:
            quaternion<< get_quaternion_links(rp_new);
            break;
        case 2:
            quaternion<< get_quaternion_hinten(rp_new);
            break;
        case 3:
            quaternion<< get_quaternion_rechts(rp_new);
            break;
        case 4:
            quaternion<< get_quaternion_vorne(rp_new);
            break;
        case 5:
            quaternion<< get_quaternion_oben(rp_new);
            break;
    }

    for(size_t i= 0; i < rot_rpys.size() ; i++){
        quaternion = rmv(rot_rpys[i],quaternion);
    }
    return quaternion;
}

MatrixXd quaternion_x_y_z(MatrixXd rpy){
    MatrixXd quaternions(6,4);
    std::vector<MatrixXd> new_rpys = rpy_to_vector(rpy);

    std::vector<MatrixXd> quats;
    MatrixXd rp_new(1,3);
    rp_new << 0.0, 0.0, 0.0;
    quats.push_back(get_quaternion_unten(rp_new));
    quats.push_back(get_quaternion_links(rp_new));
    quats.push_back(get_quaternion_hinten(rp_new));
    quats.push_back(get_quaternion_rechts(rp_new));
    quats.push_back(get_quaternion_vorne(rp_new));
    quats.push_back(get_quaternion_oben(rp_new));

    for(size_t i= 0; i < new_rpys.size() ; i++){
        for(int j = 0 ; j< 6 ; j++){
            quats[j] = rmv(new_rpys[i],quats[j]);
        }
    }
    MatrixXd quaternion_ = stdvec_to_quat_matrix(quats);
    return quaternion_;
}

MatrixXd match_deg_to_rpy_new(MatrixXd rotation_rpy, Eigen::Quaterniond grasp_q){
    std::cout << "rotation rpy : " << rotation_rpy << std::endl;

    std::vector<MatrixXd> new_rpys = rpy_to_vector(rotation_rpy);
    MatrixXd new_q = eigen_quaternion_to_matrix(grasp_q);

    for(size_t i = 0; i<new_rpys.size(); i++){
        new_q = rmv(new_rpys[i],new_q);
    }
    return new_q;

}


