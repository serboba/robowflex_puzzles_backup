//
// Created by serboba on 16.10.21.
//

#ifndef ROBOWFLEX_DART_ROTATION_HELPER_H
#define ROBOWFLEX_DART_ROTATION_HELPER_H

namespace robowflex{
    class Rotation_Helper{
        public:
        static Eigen::Quaterniond calcQuaternion(std::vector<double> vec);
        static Eigen::Quaterniond calculateCS(std::vector<double> rotation_deg);
        static std::vector<double> getDegrees(Eigen::Quaterniond q);
        static Eigen::Quaterniond rotateRight(Eigen::Quaterniond oldRotation);
        static Eigen::Quaterniond rotateLeft(Eigen::Quaterniond oldRotation);
        static Eigen::Quaterniond rotateUp(Eigen::Quaterniond oldRotation);
        static Eigen::Quaterniond rotateDown(Eigen::Quaterniond oldRotation);
        static Eigen::Quaterniond rotateYawPositive(Eigen::Quaterniond oldRotation);
        static Eigen::Quaterniond rotateYawNegative(Eigen::Quaterniond oldRotation);


        static void roundValues(std::vector<double> &values);
    };
}

#endif //ROBOWFLEX_DART_ROTATION_HELPER_H
