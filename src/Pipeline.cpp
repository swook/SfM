#include "Pipeline.hpp"

Pipeline::Pipeline(std::string _folder_path)
{
	folder_path = _folder_path;
}

void Pipeline::run()
{
	/**
	 * Stage 0: Load images from file
	 */
	Images images;
	load_images(folder_path, images);


	/**
	 * Stage 1: Detect features in loaded images
	 */
	CamFrames cam_Frames;
	DescriptorsVec descriptors_vec;
	extract_features(images, cam_Frames, descriptors_vec);

	/**
	 * Stage 2: Calculate descriptors and find image pairs through matching
	 */
}
