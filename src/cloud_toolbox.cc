# include "cloud_toolbox.h"


WWW::Cloud_Toolbox::Cloud_Toolbox(){};
WWW::Cloud_Toolbox::~Cloud_Toolbox(){};

void WWW::Cloud_Toolbox::bin2cloud(std::string bin_file, pcl::PointCloud<pcl::PointXYZI>::Ptr points){
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

void WWW::Cloud_Toolbox::pcd2cloud(std::string pcd_file, pcl::PointCloud<pcl::PointXYZI>::Ptr points){
	points->clear();
	pcl::io::loadPCDFile (pcd_file, *points);
};

void WWW::Cloud_Toolbox::cloud2bin(pcl::PointCloud<pcl::PointXYZI>::Ptr points, std::string bin_file){
	std::ofstream out_file(bin_file.c_str(), std::ios::out | std::ios::binary);
	for (int i = 0; i < points->size(); i++){
		out_file.write((char*)& points->at(i).x, sizeof(points->at(i).x));
		out_file.write((char*)& points->at(i).y, sizeof(points->at(i).y));
		out_file.write((char*)& points->at(i).z, sizeof(points->at(i).z));
		out_file.write((char *)& points->at(i).intensity, sizeof(points->at(i).intensity));
	}
	out_file.close();

};

void WWW::Cloud_Toolbox::cloud2pcd(pcl::PointCloud<pcl::PointXYZI>::Ptr points, std::string pcd_file){
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZI> (pcd_file, *points);	
};

void WWW::Cloud_Toolbox::vis_cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr points, std::shared_ptr<pcl::visualization::PCLVisualizer> viewer){




	// pcl::PointCloud<pcl::PointXYZ>::Ptr points_ (new pcl::PointCloud<pcl::PointXYZ>);

	// for (int i = 0; i < points->size(); i++){
	// 	pcl::PointXYZ point_;
	// 	point_.x = points->at(i).x;
	// 	point_.y = points->at(i).y;
	// 	point_.z = points->at(i).z;
	// 	points_->push_back(point_);
	// }

	viewer->setBackgroundColor(0,0,0);
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> single_color(points, 0, 255, 0);


	viewer->addPointCloud<pcl::PointXYZI> (points, single_color, "points");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "points");


	// while(!viewer->wasStopped()){
	// 	viewer->spinOnce(100);
	// }

};

void WWW::Cloud_Toolbox::spin(){};

void WWW::Cloud_Toolbox::image2cv(std::string img_file, cv::Mat &img){
	img = cv::imread(img_file, cv::IMREAD_UNCHANGED);
	// cv::namedWindow(img_file, cv::WINDOW_AUTOSIZE);
	// cv::imshow(img_file, img);
	// cv::waitKey();

};
void WWW::Cloud_Toolbox::label2bboxes(std::string label_file){
	std::ifstream infile;
	infile.open(label_file.data());
	std::string line, word;
	
	while (std::getline(infile, line))
	{
		WWW::Bbox tmp_bbox;
		tmp_bbox.src = line;

		// std::cout<<"------label2bboxes-----"<<std::endl;
		// std::cout<<line<<std::endl;

		std::istringstream istr(line);

		float left,right, top, bottom, h,w,l,x,y,z;
		istr >> tmp_bbox.cls_type;
		istr >> tmp_bbox.trucation;
		istr >> tmp_bbox.occlusion;
		istr >> tmp_bbox.alpha;
		istr >> left;
		istr >> top;
		istr >> right;
		istr >> bottom;
		istr >> h;
		istr >> w;
		istr >> l;
		istr >> x;
		istr >> y;
		istr >> z;
		istr >> tmp_bbox.rotation;
		istr >> tmp_bbox.score;

		tmp_bbox.pos2d << left, top, right, bottom;
		tmp_bbox.pos3d << x,y,z;
		tmp_bbox.size3d << l,w,h;


		tmp_bbox.pos2corner_3d();


		// std::cout<<"tmp_bbox.cls_type: "<<tmp_bbox.cls_type<<std::endl;
		// std::cout<<"tmp_bbox.trucation: "<<tmp_bbox.trucation<<std::endl;
		// std::cout<<"tmp_bbox.occlusion: "<<tmp_bbox.occlusion<<std::endl;
		// std::cout<<"tmp_bbox.alpha: "<<tmp_bbox.alpha<<std::endl;
		// std::cout<<"tmp_bbox.pos2d: "<<tmp_bbox.pos2d<<std::endl;
		// std::cout<<"tmp_bbox.pos3d: "<<tmp_bbox.pos3d<<std::endl;
		// std::cout<<"tmp_bbox.size3d: "<<tmp_bbox.size3d<<std::endl;
		// std::cout<<"tmp_bbox.rotation: "<<tmp_bbox.rotation<<std::endl;
		// std::cout<<"tmp_bbox.score: "<<tmp_bbox.score<<std::endl;
		bboxes.push_back(tmp_bbox);

	}
	

};
void WWW::Cloud_Toolbox::load_calib(std::string calib_file){
	std::ifstream infile;
	infile.open(calib_file.data());
	std::string line, word;
	


	for (int idx = 0; idx < 6; idx++){
		std::getline(infile, line);
		std::istringstream istr(line);
		std::string Mat_name;
		Eigen::Matrix<float, 3,4> tmp_mat;

		istr >> Mat_name;
		for (int i = 0; i < 3; i++){
			for (int j = 0; j < 4; j++){
				istr >> tmp_mat(i,j);
			}
		}

		
	}





};
void WWW::Cloud_Toolbox::axis_trans(){};
void WWW::Cloud_Toolbox::vis_ros(){};

