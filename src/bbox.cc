#include <bbox.h>




void Bbox::pos2corner_3d(){
    float l = size3d(0);
    float w = size3d(1);
    float h = size3d(2);

    Eigen::Matrix<float, 8,3> delta_pos, rotated_delta_pos;
    delta_pos<< l/2,  w/2, h, 
                l/2, -w/2, h,
               -l/2,  w/2, h,
               -l/2, -w/2, h,
                l/2,  w/2, 0,
                l/2, -w/2, 0,
               -l/2,  w/2, 0,
               -l/2, -w/2, 0;
    
    Eigen::Matrix3f R;
    R << std::cos(rotation), -std::sin(rotation), 0,
         std::sin(rotation),  std::cos(rotation), 0,
                0          ,          0        , 1;
    
    rotated_delta_pos = (R*delta_pos.transpose()).transpose();




    float x = pos3d(0);
    float y = pos3d(1);
    float z = pos3d(2);

    Eigen::Matrix<float, 8,3> pos_cen;
    pos_cen << x,y,z,
               x,y,z,
               x,y,z,
               x,y,z,
               x,y,z,
               x,y,z,
               x,y,z,
               x,y,z;

    corner3d = pos_cen + rotated_delta_pos;

};




void Bbox::corner2pos_3d(){
    pos3d = corner3d.bottomRows(4).colwise().sum()/4;
    size3d = corner3d.topRows(1)-corner3d.bottomRows(1);

    Eigen::Vector3f ori_vec;
    ori_vec = corner3d.topRows(2).colwise().sum()/2 - pos3d;
    rotation = std::atan(ori_vec(1)/ori_vec(0));
};



//TODO
void Bbox::pos2corner_2d(){};
void Bbox::corner2pos_2d(){};