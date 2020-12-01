# include "cloud_toolbox.h"


int main(){
	std::string bin_file1 = "/home/wyl/workspace/lidar_codebase/cloud_toolbox/data/samples/pandarset/000000.bin";
	std::string pcd_file1 = "/home/wyl/workspace/lidar_codebase/cloud_toolbox/data/samples/pcd/000000.pcd";
	std::string img_file  = "/home/wyl/workspace/lidar_codebase/cloud_toolbox/data/samples/semseg_res/color_mask_000000.png";
	std::string label_file= "/home/wyl/workspace/lidar_codebase/cloud_toolbox/data/samples/label_2_v3/000000.txt";
	std::string calib_file= "/home/wyl/workspace/lidar_codebase/cloud_toolbox/data/samples/calib/000000.txt";
	
	
	WWW::Cloud_Toolbox vis;


	pcl::PointCloud<pcl::PointXYZI>::Ptr points1 (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr points2 (new pcl::PointCloud<pcl::PointXYZI>);

	// std::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("Simple Cloud Visualizer"));

	// vis.bin2cloud(bin_file1, points1);
	// // vis.cloud2pcd(points1, vis.pcd_file);
	// vis.pcd2cloud(pcd_file1, points2);
	// // vis.vis_cloud(points1, viewer);
	// vis.vis_cloud(points1, viewer);

	// while(!viewer->wasStopped()){
	// 	viewer->spinOnce(100);
	// }

	// cv::Mat img;
	// vis.image2cv(img_file, img);
	// cv::namedWindow(img_file, cv::WINDOW_AUTOSIZE);
	// cv::imshow(img_file, img);
	// cv::waitKey();

	vis.label2bboxes(label_file);
	for (int i = 0; i < vis.bboxes.size(); i ++){
		WWW::Bbox tmp_bbox;
		tmp_bbox = vis.bboxes[i];
		if (tmp_bbox.cls_type != "DontCare"){
			std::cout << "-----------box-" << i <<"------"<<std::endl;
			// std::cout<<"tmp_bbox.pos3d: "<<tmp_bbox.pos3d<<std::endl;
			// std::cout<<"tmp_bbox.size3d: "<<tmp_bbox.size3d<<std::endl;
			// std::cout<<"tmp_bbox.rotation: "<<tmp_bbox.rotation<<std::endl;
			// std::cout<<"tmp_bbox.corner3d: "<<tmp_bbox.corner3d<<std::endl;
			// tmp_bbox.corner2pos_3d();
			// std::cout<<"tmp_bbox.pos3d: "<<tmp_bbox.pos3d<<std::endl;
			// std::cout<<"tmp_bbox.size3d: "<<tmp_bbox.size3d<<std::endl;
			// std::cout<<"tmp_bbox.rotation: "<<tmp_bbox.rotation<<std::endl;


			std::cout<<"tmp_bbox.cls_type: "<<std::endl<<tmp_bbox.cls_type<<std::endl;
			std::cout<<"tmp_bbox.trucation: "<<std::endl<<tmp_bbox.trucation<<std::endl;
			std::cout<<"tmp_bbox.occlusion: "<<std::endl<<tmp_bbox.occlusion<<std::endl;
			std::cout<<"tmp_bbox.alpha: "<<std::endl<<tmp_bbox.alpha<<std::endl;
			std::cout<<"tmp_bbox.pos2d: "<<std::endl<<tmp_bbox.pos2d<<std::endl;
			std::cout<<"tmp_bbox.pos3d: "<<std::endl<<tmp_bbox.pos3d<<std::endl;
			std::cout<<"tmp_bbox.size3d: "<<std::endl<<tmp_bbox.size3d<<std::endl;
			std::cout<<"tmp_bbox.rotation: "<<std::endl<<tmp_bbox.rotation<<std::endl;
			std::cout<<"tmp_bbox.corner3d: "<<std::endl<<tmp_bbox.corner3d<<std::endl;
			std::cout<<"tmp_bbox.score: "<<std::endl<<tmp_bbox.score<<std::endl;
		}

	}



	
	return 0;
}