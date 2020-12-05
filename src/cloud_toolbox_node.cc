# include "cloud_toolbox.h"


#include <fstream>
#include <string>

int main(){
	int id1 =8;
	int id2 = id1/6;

	std::cout <<"id2"<<id2<<std::endl;
	std::stringstream ss1, ss2;
	ss1 << setw(6)<<setfill('0')<< id1;
	ss2 <<setw(6)<<setfill('0')<< id2;

	std::string id1_str = ss1.str();
	std::string id2_str = ss2.str();
	std::string bin_file1 = "/home/wyl/workspace/lidar_codebase/cloud_toolbox/data/samples/pandarset/"+id1_str+".bin";
	// std::string pcd_file1 = "/home/wyl/workspace/lidar_codebase/cloud_toolbox/data/samples/pcd/"+id1_str +".pcd";
	std::string img_file  = "/home/wyl/workspace/lidar_codebase/cloud_toolbox/data/samples/semseg_res/color_mask_"+id1_str +".png";
	std::string label_file= "/home/wyl/workspace/lidar_codebase/cloud_toolbox/data/samples/label_2_v3/"+id1_str +".txt";
	std::string calib_file= "/home/wyl/workspace/lidar_codebase/cloud_toolbox/data/samples/calib/"+id2_str +".txt";
	
	std::cout<< "id1_str: "<<id1_str<<std::endl;
	std::cout<< "id2_str: "<<id2_str<<std::endl;
	WWW::Cloud_Toolbox vis;
	vis.calib_id = id1%6;

	std::cout<< "calib_id: "<<vis.calib_id<<std::endl;
	pcl::PointCloud<pcl::PointXYZI>::Ptr points1 (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr points2 (new pcl::PointCloud<pcl::PointXYZI>);

	std::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("Simple Cloud Visualizer"));
	cv::Mat img;
	vis.load_cloud_bin(bin_file1, points1);
	vis.load_bboxes(label_file);
	vis.load_calib(calib_file);
	vis.load_image(img_file, img);
	vis.axis_trans(points1, points2);

	vis.vis_cloud(points2, viewer, true);

	while(!viewer->wasStopped()){
		viewer->spinOnce(10000);
	}


	// cv::namedWindow(img_file, cv::WINDOW_AUTOSIZE);
	// cv::imshow(img_file, img);
	// cv::waitKey();

	// for (int i = 0; i < vis.bboxes.size(); i ++){
	// 	WWW::Bbox tmp_bbox;
	// 	tmp_bbox = vis.bboxes[i];
	// 	if (tmp_bbox.cls_type != "DontCare"){
	// 		std::cout << "-----------box-" << i <<"------"<<std::endl;
	// 		// std::cout<<"tmp_bbox.pos3d: "<<tmp_bbox.pos3d<<std::endl;
	// 		// std::cout<<"tmp_bbox.size3d: "<<tmp_bbox.size3d<<std::endl;
	// 		// std::cout<<"tmp_bbox.rotation: "<<tmp_bbox.rotation<<std::endl;
	// 		// std::cout<<"tmp_bbox.corner3d: "<<tmp_bbox.corner3d<<std::endl;
	// 		// tmp_bbox.corner2pos_3d();
	// 		// std::cout<<"tmp_bbox.pos3d: "<<tmp_bbox.pos3d<<std::endl;
	// 		// std::cout<<"tmp_bbox.size3d: "<<tmp_bbox.size3d<<std::endl;
	// 		// std::cout<<"tmp_bbox.rotation: "<<tmp_bbox.rotation<<std::endl;


	// 		std::cout<<"tmp_bbox.cls_type: "<<std::endl<<tmp_bbox.cls_type<<std::endl;
	// 		std::cout<<"tmp_bbox.trucation: "<<std::endl<<tmp_bbox.trucation<<std::endl;
	// 		std::cout<<"tmp_bbox.occlusion: "<<std::endl<<tmp_bbox.occlusion<<std::endl;
	// 		std::cout<<"tmp_bbox.alpha: "<<std::endl<<tmp_bbox.alpha<<std::endl;
	// 		std::cout<<"tmp_bbox.pos2d: "<<std::endl<<tmp_bbox.pos2d<<std::endl;
	// 		std::cout<<"tmp_bbox.pos3d: "<<std::endl<<tmp_bbox.pos3d<<std::endl;
	// 		std::cout<<"tmp_bbox.size3d: "<<std::endl<<tmp_bbox.size3d<<std::endl;
	// 		std::cout<<"tmp_bbox.rotation: "<<std::endl<<tmp_bbox.rotation<<std::endl;
	// 		std::cout<<"tmp_bbox.corner3d: "<<std::endl<<tmp_bbox.corner3d<<std::endl;
	// 		std::cout<<"tmp_bbox.score: "<<std::endl<<tmp_bbox.score<<std::endl;
	// 	}

	// }

	// for (int i = 0; i< vis.calib.P.size(); i++){
	// 	std::cout<<"-----------P"<<i<<"------------"<<std::endl;
	// 	std::cout<<vis.calib.P[i]<<std::endl;
	// }
	// std::cout<<"-----------R0_rect------------"<<std::endl;
	// std::cout<<vis.calib.R0_rect<<std::endl;
	// for (int i = 0; i< vis.calib.Tr_velo_to_cam.size(); i++){
	// 	std::cout<<"-----------Tr_vel_to_cam"<<i<<"------------"<<std::endl;
	// 	std::cout<<vis.calib.Tr_velo_to_cam[i]<<std::endl;
	// }
	// std::cout<<"-----------Tr_imu_to_velo------------"<<std::endl;
	// std::cout<<vis.calib.Tr_imu_to_velo<<std::endl;



	
	return 0;
}