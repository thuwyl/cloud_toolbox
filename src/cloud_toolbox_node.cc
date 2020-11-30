# include "cloud_toolbox.h"


int main(){
	std::string bin_file1 = "/home/wyl/workspace/Hesai/samples/pandarset/000006.bin";
	std::string pcd_file1 = "/home/wyl/workspace/Hesai/samples/pcd/000000.pcd";
	std::string img_file  =  "/home/wyl/workspace/Hesai/samples/semseg_res/color_mask_000000.png";
	std::string label_file = "/home/wyl/workspace/Hesai/samples/label_2_v3/000000.txt";
	
	
	Cloud_Toolbox vis(bin_file1, pcd_file1);


	pcl::PointCloud<pcl::PointXYZI>::Ptr points1 (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr points2 (new pcl::PointCloud<pcl::PointXYZI>);

	std::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("Simple Cloud Visualizer"));

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
		Bbox tmp_bbox;
		tmp_bbox = vis.bboxes[i];
		if (tmp_bbox.cls_type != "DontCare"){
			std::cout << "-----------box-" << i <<"------"<<std::endl;
			std::cout<<"tmp_bbox.cls_type: "<<tmp_bbox.cls_type<<std::endl;
			std::cout<<"tmp_bbox.trucation: "<<tmp_bbox.trucation<<std::endl;
			std::cout<<"tmp_bbox.occlusion: "<<tmp_bbox.occlusion<<std::endl;
			std::cout<<"tmp_bbox.alpha: "<<tmp_bbox.alpha<<std::endl;
			std::cout<<"tmp_bbox.pos2d: "<<tmp_bbox.pos2d<<std::endl;
			std::cout<<"tmp_bbox.pos3d: "<<tmp_bbox.pos3d<<std::endl;
			std::cout<<"tmp_bbox.size3d: "<<tmp_bbox.size3d<<std::endl;
			std::cout<<"tmp_bbox.rotation: "<<tmp_bbox.rotation<<std::endl;
			std::cout<<"tmp_bbox.score: "<<tmp_bbox.score<<std::endl;
		}

	}



	
	return 0;
}