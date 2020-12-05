#include <Eigen/Dense>

#include <iostream>
#include <fstream>
#include <string>
#include <cassert>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
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
    void load_cloud_bin(std::string bin_file, pcl::PointCloud<pcl::PointXYZI>::Ptr points); // load .bin file, to pcl pointcloud format
    void load_cloud_pcd(std::string pcd_file, pcl::PointCloud<pcl::PointXYZI>::Ptr points); // load .pcd file, to pcl pointcloud format
    void save_cloud_bin(pcl::PointCloud<pcl::PointXYZI>::Ptr points, std::string bin_file); // save pointcloud to .bin file
    void save_cloud_pcd(pcl::PointCloud<pcl::PointXYZI>::Ptr points, std::string pcd_file); // save pointcloud to .pcd file
    void vis_cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr points, std::shared_ptr<pcl::visualization::PCLVisualizer> viewer, bool vis_bboxes); //visualize the pointcloud

    void load_image(std::string img_file, cv::Mat &img);
    void load_bboxes(std::string label_file);
    void load_calib(std::string calib_file);
    void axis_trans(pcl::PointCloud<pcl::PointXYZI>::Ptr points, pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_points);



    void select_points(cv::Mat &img, pcl::PointCloud<pcl::PointXYZI>::Ptr points);
    void select_bboxes(cv::Mat &img);
    //input
    std::string bin_file;
    std::string image_file;
    std::string label_file;
    std::string calib_file;
    
    int calib_id;

    std::vector<WWW::Bbox> bboxes;
    WWW::Calib calib;


    //output
    std::string pcd_file;
};}