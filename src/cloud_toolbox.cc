# include "cloud_toolbox.h"


int main(){
	std::string bin_file1 = "/home/wyl/workspace/Hesai/samples/pandarset/000000.bin";
	std::string pcd_file1 = "/home/wyl/workspace/Hesai/samples/pcd/000000.pcd";
	Cloud_Toolbox vis(bin_file1, pcd_file1);


	pcl::PointCloud<pcl::PointXYZI>::Ptr points1 (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr points2 (new pcl::PointCloud<pcl::PointXYZI>);
	// vis.bin2cloud(vis.bin_file, points1);
	// vis.cloud2pcd(points1, vis.pcd_file);
	vis.pcd2cloud(vis.pcd_file, points2);
	vis.vis_cloud(points2);
	
	return 0;
}