#pragma once

#include "opencv2/opencv.hpp"

#include "structures.hpp"
#include "Associativity.hpp"

class Pipeline {

public:
	/**
	 * Constructors
	 */
	Pipeline(std::string folder_path);

	const cv::Mat cameraMatrix;
	const cv::Mat distCoeffs;

	/**
	 * Run pipeline
	 */
	void run();

private:
	std::string folder_path;

	void load_images(std::string _folder_path, Images& images);

	// detect features and extract descriptors. features are stored in CamFrames::key_points
	void extract_features(const Images& images, CamFrames& cam_Frames, DescriptorsVec& descriptors_vec);
	// find matching features and valid matching frames
	void find_matching_pairs(
		const Images& images,
		const CamFrames& camframes,
		const DescriptorsVec& descriptors_vec,
		ImagePairs& pairs);
	// get pairwise camera pose from matching frames
	void register_camera(ImagePairs& pairs,CamFrames& cam_Frames);
	// 3D-2D registration
	// bool ransacRegistration(ImagePair* pair);

	const int build_spanning_tree(const ImagePairs& pairs, Associativity& assocMat,
		Associativity& tree);

	void glo_cam_poses(Images& images, CameraPoses& cameraPoses, const ImagePairs& pairs,
		Associativity& tree);

	// find and cluster depth points from local camera frame to global camera frame
	void find_clusters(Associativity& assocMat,const CameraPoses& cameraPoses,
		const CamFrames& camFrames,PointClusters& pointClusters,PointMap& pointMap);

	void find_CoM(const PointClusters& pointClusters,PointCloud& pointCloud);

	void bundle_adjustment(
	const PointMap& pointMap,
	const CameraPoses& poses,
	const CamFrames& camFrames,
	PointCloud pointCloud);
};
