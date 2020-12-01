#include <iostream>
#include <Eigen/Dense>

namespace WWW{
// axis definition:
// front-x  left-y  up-z

class Bbox
{
public:
    Bbox();
    ~Bbox();
// private:
    std::string src;
    std::string cls_type;
    int cls_id;
    int track_id;
    float trucation;
    float occlusion; //0:fully visible 1:partly occluded 2:largely occluded 3:unknown


    Eigen::Vector4f pos2d;  //left top right bottom
    float alpha;
    Eigen::Matrix<float, 4,2> corner2d;
    // corner2d:
    //      1-------2
    //      |       |
    //      |       |
    //      3-------4
    

    Eigen::Vector3f pos3d;  //bottom center of the 3d bbox (x,y,z)
    Eigen::Vector3f size3d; //l,w,h
    float rotation;  //real orientation respect to the x-axis, inverse clock is positive
    Eigen::Matrix<float, 8,3> corner3d;

    float score;
    // corner3d:
    //          1-------2
    //        / |     / |
    //      3---|---4   |
    //      |   5---|---6
    //      | /     | /
    //      7-------8
    void pos2corner_2d();
    void corner2pos_2d();

    void pos2corner_3d();
    void corner2pos_3d();



};
}