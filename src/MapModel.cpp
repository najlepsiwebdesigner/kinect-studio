#include "MapModel.h"

// void MapModel::insertFrame(Frame & frame) {
// 	std::vector<int> inserted_feature_indices;

// 	for (int i = 0; i < frame.feature_cloud->points.size();i++) {
// 		feature_cloud->points.push_back(frame.feature_cloud->points[i]);
// 		inserted_feature_indices.push_back(feature_cloud->points.size());
// 		descriptors.push_back(frame.descriptors.row(i));
// 	}

// 	camera_poses.push_back(frame.transform_visual);
// 	camera_poses_to_indices.push_back(inserted_feature_indices);
// }