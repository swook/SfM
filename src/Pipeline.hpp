#pragma once

#include "structures.hpp"

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

	void load_images(std::string _folder_path, Images& images);

	// detect features and extract descriptors. features are stored in CamFrames::key_points
	void extract_features(const Images& images, CamFrames& cam_Frames, DescriptorsVec& descriptors_vec);

	void find_matching_pairs(
		const Images& images,
		const CamFrames& camframes,
		const DescriptorsVec& descriptors_vec,
		ImagePairs& pairs);
};
