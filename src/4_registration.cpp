#include "opencv2/opencv.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
using namespace cv;

#include "Pipeline.hpp"

void Pipeline::register_camera(ImagePairs& pairs,const Images& images,CamFrames& cam_Frames){
	std::cout << "number of pairs " << pairs.size() << std::endl;
	// get initial 3D points from the first camera
	for(auto it = pairs.begin(); it != pairs.end(); it++){
		int i = (it -> pair_index).first;
		int j = (it -> pair_index).second;
		std::vector<Point2f> keyPoints1 = it -> matched_points.first;
		std::vector<Point2f> keyPoints2 = it -> matched_points.second;
		std::cout << "number of matches " << keyPoints1.size() << ", " << keyPoints2.size() << std::endl;
		// get depth map of the first camera frame
		Mat depth_img1 = images.at(i).dep;
		
		// get depth value of all matched keypoints in image1
		std::vector<Point3f> points3D;
		std::vector<Point2f> valid_keyPoints;
		for(std::size_t k = 0; k < keyPoints1.size(); ++k){
			float x = keyPoints1[k].x;
			float y = keyPoints1[k].y;
			float d = (float) depth_img1.at<uint16_t>((int)x,(int)y);
			// skip invalid depth
			if (d == 0) continue;
			Mat new_point = d * (Mat_<float>(1,3) << x,y,1) * cameraMatrix.inv().t();
			
			valid_keyPoints.push_back(keyPoints2[k]);
			points3D.push_back(Point3f(new_point));
		}
		Mat rvec,tvec, R;
		solvePnPRansac(points3D,valid_keyPoints,cameraMatrix,distCoeffs,rvec,tvec);
		Rodrigues(rvec,R);
		it -> R = R;
		it -> t = tvec;
	}
}