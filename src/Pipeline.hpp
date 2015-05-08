#pragma once

#include "structures.hpp"
#include "opencv2/opencv.hpp"

class Pipeline {

public:
	/**
	 * Constructors
	 */
	Pipeline(std::string folder_path);
	/**
	 * Run pipeline
	 */
	void run();

private:
	std::string folder_path;
	const cv::Mat cameraMatrix;
	const cv::Mat distCoeffs;
	MatchMap match_map;

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

	void build_spanning_tree(const ImagePairs& pairs, Associativity& assocMat);
};
