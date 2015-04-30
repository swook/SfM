#pragma once

#include "structures.hpp"
#include "opencv2/opencv.hpp"

class Pipeline {

public:
	/**
	 * Constructors
	 */
	Pipeline(std::string folder_path);

	// Focal length 524px
	// Principal point 316.7 238.5
	// Distortion coefficients kc1 kc2 kc3 kc4 = 0.2402 -0.6861 -0.0015 0.0003
	const float intrinsic_array[3][3] = {{524,0,316.7},{0,524,238.5},{0,0,1}};
	const float distcoeff[4] = {0.2402,-0.6861,-0.0015,0.0003};

	cv::Mat distcoeff_matrix();

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
	void register_camera(const ImagePairs& pairs,CamFrames& cam_Frames);
};
