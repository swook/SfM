#include "Pipeline.hpp"
#include "opencv2/opencv.hpp"

using namespace cv;

Pipeline::Pipeline(std::string _folder_path)
{
	folder_path = _folder_path;
}

Mat Pipeline::distcoeff_matrix()
{
	Mat output(1,4,CV_32F,distcoeff);
	return output;
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
	ImagePairs image_pairs;
	find_matching_pairs(images, cam_Frames, descriptors_vec, image_pairs);
}
