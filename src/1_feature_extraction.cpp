#include "opencv2/opencv.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
using namespace cv;

#include "Pipeline.hpp"

void Pipeline::extract_features(const Images& images,CamFrames& cam_Frames,DescriptorsVec& descriptors_vec)
{
	// create a sift detector
	// const int feature_num = 50;
	// Ptr<Feature2D> sift_detector = SIFT::create(feature_num);
	
	// create brist detector
	Ptr<Feature2D> brisk_detector = BRISK::create();

	// detect features in a loop
	for (int i = 0; i < images.size(); ++i)
	{
		KeyPoints key_points;
		Descriptors descriptors;
		// detects and computes descriptors
		brisk_detector->detectAndCompute(images[i].rgb,noArray(),key_points,descriptors);
		// wrap keypoints to cam_Frame and add in to cam_Frames 
		cam_Frames.push_back((CamFrame) {key_points});
		descriptors_vec.push_back(descriptors);
	}
}