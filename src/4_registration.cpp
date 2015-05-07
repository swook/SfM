#include "opencv2/opencv.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
using namespace cv;

#include "Pipeline.hpp"

void Pipeline::register_camera(ImagePairs& pairs,const Images& images,CamFrames& cam_Frames){
	 // create camera matrix from camera params

	 // Focal length 524px
	 // Principal point 316.7 238.5
	 // Distortion coefficients kc1 kc2 kc3 kc4 = 0.2402 -0.6861 -0.0015 0.0003


	 // get initial 3D points from the first camera

	 // register camera 3D-2D
	 // solvePnPRansac(InputArray objectPoints, InputArray imagePoints, InputArray cameraMatrix,
	 // 	InputArray distCoeffs, OutputArray rvec, OutputArray tvec, bool useExtrinsicGuess=false,
	 // 	int iterationsCount=100, float reprojectionError=8.0, double confidence=0.99, OutputArray inliers=noArray(), int flags=SOLVEPNP_ITERATIVE )

	/*
		Camera i-to-j

		InputArray objectPoints			Keypoints in 3D space wrt camera i
		InputArray imagePoints			Keypoints on image of camera j
		InputArray cameraMatrix
		InputArray distCoeffs
		OutputArray rvec 			Rotation:    model-to-camera
		OutputArray tvec			Translation: model-to-camera

		bool useExtrinsicGuess=false
		int iterationsCount=100
		float reprojectionError=8.0

		int minInliersCount=100 		RANSAC
		OutputArray inliers=noArray(), 		RANSAC
		int flags=ITERATIVE			PnP flags
	*/

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
