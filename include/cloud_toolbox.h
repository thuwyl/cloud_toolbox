#include <Eigen/Dense>

#include <iostream>
#include <fstream>
#include <string>
#include <cassert>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <memory>

#include <opencv2/opencv.hpp>


#include <bbox.h>
#include <calib.h>
#include <math.h>

#include <vector>

namespace WWW{
class Cloud_Toolbox
{
public:
    Cloud_Toolbox();
    ~Cloud_Toolbox();
    void spin();

    // private:
    void bin2cloud(std::string bin_file, pcl::PointCloud<pcl::PointXYZI>::Ptr points); // load .bin file, to pcl pointcloud format
    void pcd2cloud(std::string pcd_file, pcl::PointCloud<pcl::PointXYZI>::Ptr points); // load .pcd file, to pcl pointcloud format
    void cloud2bin(pcl::PointCloud<pcl::PointXYZI>::Ptr points, std::string bin_file); // save pointcloud to .bin file
    void cloud2pcd(pcl::PointCloud<pcl::PointXYZI>::Ptr points, std::string pcd_file); // save pointcloud to .pcd file
    void vis_cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr points, std::shared_ptr<pcl::visualization::PCLVisualizer> viewer); //visualize the pointcloud

    void image2cv(std::string img_file, cv::Mat &img);
    void label2bboxes(std::string label_file);
    void load_calib(std::string calib_file);
    void axis_trans();
    void vis_ros();

    //input
    std::string bin_file;
    std::string image_file;
    std::string label_file;
    std::string calib_file;

    std::vector<WWW::Bbox> bboxes;

    WWW::Calib calib;

    //output
    std::string pcd_file;
};}