#ifndef MAPMODEL_H
#define MAPMODEL_H

#include "globals.h"

#include <pcl/common/transforms.h>

using namespace app;

class MapModel {
public:
	// these two structures has the same indices 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr feature_cloud;
	cv::Mat descriptors;
	std::vector<cv::KeyPoint> keypoints;
	// these contain all camera poses from visual odometry
	std::vector<Eigen::Affine3f> camera_poses;
	std::vector<int> indices_hit_count;

	// will be set to true after first successful insertion 
	bool is_ready = false;

	// these maps camera poses to points
	std::vector<std::vector<int> > camera_poses_to_indices;

	MapModel(): 
		feature_cloud(new pcl::PointCloud<pcl::PointXYZRGB>)
	{

	}

	void insertFrame(Frame & frame) {
		std::vector<int> inserted_feature_indices;

		// std::cout << frame.model_indices.size() << std::endl;
		// std::cout << frame.feature_cloud->points.size() << std::endl;

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_feature_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
        pcl::transformPointCloud<pcl::PointXYZRGB>(*frame.feature_cloud, *transformed_feature_cloud, frame.transform_visual);
        *frame.feature_cloud = *transformed_feature_cloud;

        // std::cout << "feature points: " << feature_cloud->points.size() << std::endl;
        // std::cout << "matches: " << frame.good_feature_matches.size() << std::endl;

		int fused_points = 0;
		for (int i = 0; i < frame.feature_cloud->points.size();i++) {
			// fuse this point
			int current_model_index = frame.model_indices[i];
			if (current_model_index != -1) {
				// raise counter on this point hit counter
				indices_hit_count[current_model_index]++;

				// // average of current measurement and previous value
				// feature_cloud->points[current_model_index].x = (feature_cloud->points[current_model_index].x + frame.feature_cloud->points[i].x)/2;
				// feature_cloud->points[current_model_index].y = (feature_cloud->points[current_model_index].y + frame.feature_cloud->points[i].y)/2;
				// feature_cloud->points[current_model_index].z = (feature_cloud->points[current_model_index].z + frame.feature_cloud->points[i].z)/2;

				// // // new value
				// feature_cloud->points[current_model_index].x = frame.feature_cloud->points[i].x;
				// feature_cloud->points[current_model_index].y = frame.feature_cloud->points[i].y;
				// feature_cloud->points[current_model_index].z = frame.feature_cloud->points[i].z;

				// running average of feature position
				feature_cloud->points[current_model_index].x = 
					(feature_cloud->points[current_model_index].x * (indices_hit_count[current_model_index] -1) 
					+ frame.feature_cloud->points[i].x) 
					/ indices_hit_count[current_model_index];

				feature_cloud->points[current_model_index].y = 
					(feature_cloud->points[current_model_index].y * (indices_hit_count[current_model_index] -1) 
					+ frame.feature_cloud->points[i].y) 
					/ indices_hit_count[current_model_index];
					
				feature_cloud->points[current_model_index].z = 
					(feature_cloud->points[current_model_index].z * (indices_hit_count[current_model_index] -1) 
					+ frame.feature_cloud->points[i].z) 
					/ indices_hit_count[current_model_index];

				descriptors.row(current_model_index) = frame.descriptors.row(i);
				keypoints[current_model_index] = frame.keypoints[i];
				fused_points++;
				inserted_feature_indices.push_back(current_model_index);
			}
			else {
				feature_cloud->points.push_back(frame.feature_cloud->points[i]);
				descriptors.push_back(frame.descriptors.row(i));
				keypoints.push_back(frame.keypoints[i]);	
				indices_hit_count.push_back(0);
				inserted_feature_indices.push_back(feature_cloud->points.size() - 1);
			}
		}

        // std::cout << "feature points: " << frame.feature_cloud->points.size() << std::endl;
        // std::cout << "fused points: " << fused_points << std::endl;

		camera_poses.push_back(frame.transform_visual);
		camera_poses_to_indices.push_back(inserted_feature_indices);

		if (!is_ready) is_ready = true;
	}

	Frame getPredictedFrame() {

		// // statistics about multi features
		// int multihit = 0;
		// int more_than_10 = 0;

		// for (auto & hit_count : indices_hit_count) {
		// 	if (hit_count != 0) {
		// 		multihit++;

		// 		if (hit_count > 10) {
		// 			more_than_10++;
		// 		}
		// 	}
		// }
		// std::cout 
		// 	<< "Percentage of multihit: " 
		// 	<< (multihit*100) / indices_hit_count.size() 
		// 	<< "% of indices total: " 
		// 	<< indices_hit_count.size() 
		// 	<< " more than 10: " 
		// 	<< more_than_10
		// 	<< std::endl;





		Frame predicted_frame;
		std::vector<int> feature_point_indices;

		// select last received frame
		feature_point_indices = camera_poses_to_indices[camera_poses.size() - 1];


		// std::cout << feature_point_indices.size() << std::endl;
		// select last 30 poses, if available
		int used_previous_poses_count = 10;
		int used_frames_count = (camera_poses.size() > used_previous_poses_count ? used_previous_poses_count : camera_poses.size());
		for (int i = camera_poses.size() - 1; i > camera_poses.size() - used_frames_count && i > 0; i=i-2) {
			feature_point_indices.insert(
				feature_point_indices.end(),
				camera_poses_to_indices[i].begin(),
				camera_poses_to_indices[i].end()
			);
		}
		

		// std::cout << "camera_poses_size: " << camera_poses.size() << std::endl;
		// std::cout << "feature_cloud_size: " << feature_cloud->points.size() << std::endl;
		// std::cout << "feature_point_indices_size: " << feature_point_indices.size() << std::endl;
		// std::cout << "camera_poses_to_indices_size: " << camera_poses_to_indices.size() << std::endl;

		// int good_indices_counter = 0;
		// for (int i = 0; i < feature_point_indices.size(); i++) {
		// 	if (indices_hit_count[i] > 0) {
		// 		good_indices_counter++;
		// 	}
		// }		
		// std::cout << "multi hit indices count: " << good_indices_counter << std::endl;
		// bool only_multi_features = good_indices_counter > 0;

// std::cout << "feature point in predicted frame count: " <<  feature_point_indices.size() << std::endl;
sort( feature_point_indices.begin(), feature_point_indices.end() );
feature_point_indices.erase( unique( feature_point_indices.begin(), feature_point_indices.end() ), feature_point_indices.end() );
// std::cout << "feature point in predicted frame  after reduction count: " <<  feature_point_indices.size() << std::endl;

		for (int i = 0; i < feature_point_indices.size(); i++) {
			// if (only_multi_features && indices_hit_count[feature_point_indices[i]] < 1 && i % 2 == 0) continue; 
		
			predicted_frame.cloud->points.push_back(feature_cloud->points[feature_point_indices[i]]);
			predicted_frame.descriptors.push_back(descriptors.row(feature_point_indices[i]));
			predicted_frame.keypoints.push_back(keypoints[feature_point_indices[i]]);
			predicted_frame.model_indices.push_back(feature_point_indices[i]);
		}


		// predicted frame is in global coordinates, we need to transform it to last observed camera pose via inverse transform of this camera pose
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
        pcl::transformPointCloud<pcl::PointXYZRGB>(*predicted_frame.cloud, *transformed_cloud, camera_poses[camera_poses.size() - 1].inverse());
        *predicted_frame.cloud = *transformed_cloud;

        // std::cout << "Predicted frame transform is: " << camera_poses[camera_poses.size() - 1].inverse().matrix() << std::endl;

		predicted_frame.is_predicted = true;
		return predicted_frame;
	}
};




#endif //MAPMODEL_H