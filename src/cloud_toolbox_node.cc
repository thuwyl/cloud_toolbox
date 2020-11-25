# include "cloud_toolbox.h"


void Cloud_Toolbox::bin2cloud(std::string bin_file, pcl::PointCloud<pcl::PointXYZI>::Ptr points){
    std::fstream input(bin_file.c_str(), std::ios::in | std::ios::binary);
    if(!input.good()){
		std::cerr << "Could not read file: " << bin_file << std::endl;
		exit(EXIT_FAILURE);
	}
	input.seekg(0, std::ios::beg);

	points->clear();
    int i;
	for (i=0; input.good() && !input.eof(); i++) {
		pcl::PointXYZI point;
		input.read((char *) &point.x, 3*sizeof(float));
		input.read((char *) &point.intensity, sizeof(float));
		points->push_back(point);
	}
	input.close();
	std::cout << "Read .bin point cloud with " << i << " points." << std::endl;
};

void Cloud_Toolbox::pcd2cloud(std::string pcd_file, pcl::PointCloud<pcl::PointXYZI>::Ptr points){
	points->clear();
	pcl::io::loadPCDFile (pcd_file, *points);
};

void Cloud_Toolbox::cloud2bin(pcl::PointCloud<pcl::PointXYZI>::Ptr points, std::string bin_file){
	std::ofstream out_file(bin_file.c_str(), std::ios::out | std::ios::binary);
	for (int i = 0; i < points->size(); i++){
		out_file.write((char*)& points->at(i).x, sizeof(points->at(i).x));
		out_file.write((char*)& points->at(i).y, sizeof(points->at(i).y));
		out_file.write((char*)& points->at(i).z, sizeof(points->at(i).z));
		out_file.write((char *)& points->at(i).intensity, sizeof(points->at(i).intensity));
	}
	out_file.close();

};

void Cloud_Toolbox::cloud2pcd(pcl::PointCloud<pcl::PointXYZI>::Ptr points, std::string pcd_file){
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZI> (pcd_file, *points);	
};

void Cloud_Toolbox::vis_cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr points){
	pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
	viewer.showCloud(points);
	while (!viewer.wasStopped()){};
};

void Cloud_Toolbox::spin(){};
void Cloud_Toolbox::image2cv(){};
void Cloud_Toolbox::label2bbox(){};
void Cloud_Toolbox::calib2matrix(){};
void Cloud_Toolbox::axis_trans(){};
void Cloud_Toolbox::vis_ros(){};

