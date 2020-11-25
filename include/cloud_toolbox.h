#include <Eigen/Dense>

#include <iostream>
#include <fstream>

// #include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

class Cloud_Toolbox{
public:
    Cloud_Toolbox(std::string bin_file, std::string pcd_file):bin_file(bin_file),pcd_file(pcd_file){};
    ~Cloud_Toolbox(){};
    void spin();

// private:
    void bin2cloud(std::string bin_file, pcl::PointCloud<pcl::PointXYZI>::Ptr points); // load .bin file, to pcl pointcloud format
    void pcd2cloud(std::string pcd_file, pcl::PointCloud<pcl::PointXYZI>::Ptr points); // load .pcd file, to pcl pointcloud format
    void cloud2bin(pcl::PointCloud<pcl::PointXYZI>::Ptr points, std::string bin_file); // save pointcloud to .bin file
    void cloud2pcd(pcl::PointCloud<pcl::PointXYZI>::Ptr points, std::string pcd_file); // save pointcloud to .pcd file
    void vis_cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr points); //visualize the pointcloud


    void image2cv();
    void label2bbox();
    void calib2matrix();
    void axis_trans();
    void vis_ros();


    //input
    std::string bin_file;
    std::string image_file;
    std::string label_file;
    std::string calib_file;
    
    //output
    std::string pcd_file;

};