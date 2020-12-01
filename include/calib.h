#include <iostream>
#include <Eigen/Dense>
#include <vector>

namespace WWW{

class Calib {
public:
    Calib();
    ~Calib();

    std::vector<Eigen::Matrix<float, 3,4>> Tr_velo_to_cam;
    std::vector<Eigen::Matrix4f> P;
    Eigen::Matrix4f R0_rect;
    Eigen::Matrix4f Tr_imu_to_velo;
    
};
}