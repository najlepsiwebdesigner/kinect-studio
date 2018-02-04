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

	// these maps camera poses to points
	std::vector<std::vector<int> > camera_poses_to_indices;

	MapModel(): 
		feature_cloud(new pcl::PointCloud<pcl::PointXYZRGB>)
	{

	}

	void insertFrame(Frame & frame) {
		std::vector<int> inserted_feature_indices;

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_feature_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
        pcl::transformPointCloud<pcl::PointXYZRGB>(*frame.feature_cloud, *transformed_feature_cloud, frame.transform_visual);
        *frame.feature_cloud = *transformed_feature_cloud;

		for (int i = 0; i < frame.feature_cloud->points.size();i++) {
			feature_cloud->points.push_back(frame.feature_cloud->points[i]);
			descriptors.push_back(frame.descriptors.row(i));
			keypoints.push_back(frame.keypoints[i]);
			
			inserted_feature_indices.push_back(feature_cloud->points.size() - 1);
		}

		camera_poses.push_back(frame.transform_visual);
		// std::cout << "Indices to be inserted: " << inserted_feature_indices.size() << std::endl;
		camera_poses_to_indices.push_back(inserted_feature_indices);
	}

	Frame getPredictedFrame() {
		Frame predicted_frame;

		std::vector<int> feature_point_indices;

		// select last received frame
//		feature_point_indices = camera_poses_to_indices[camera_poses.size() - 1];

		int used_frames_count = (camera_poses.size() > 30 ? 30 : camera_poses.size());
		for (int i = camera_poses.size() - 1; i > camera_poses.size() - used_frames_count && i > 0; i--) {
            std::cout << i << std::endl;
			feature_point_indices.insert(
				feature_point_indices.end(),
				camera_poses_to_indices[i].begin(),
				camera_poses_to_indices[i].end()
			);

			// std::cout << "features collecting in prediction: " << feature_point_indices.size() << std::endl;
		}

		// std::cout << "camera_poses_size: " << camera_poses.size() << std::endl;
		// std::cout << "feature_cloud_size: " << feature_cloud->points.size() << std::endl;
		// std::cout << "feature_point_indices_size: " << feature_point_indices.size() << std::endl;
		// std::cout << "camera_poses_to_indices_size: " << camera_poses_to_indices.size() << std::endl;

		for (int i = 0; i < feature_point_indices.size(); i++) {
			predicted_frame.cloud->points.push_back(feature_cloud->points[feature_point_indices[i]]);
			predicted_frame.descriptors.push_back(descriptors.row(feature_point_indices[i]));
			predicted_frame.keypoints.push_back(keypoints[feature_point_indices[i]]);
		}

		return predicted_frame;
	}
};




#endif //MAPMODEL_H