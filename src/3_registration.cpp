#include "opencv2/opencv.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
using namespace cv;

#include "Pipeline.hpp"
#include "util.hpp"

void Pipeline::register_camera(ImagePairs& pairs,CamFrames& cam_Frames){
	Logger _log("Step 3 (register)");

	// ---------Parameters for solvePnPRansac----------------//
	bool useExtrinsicGuess = false;
	int iterationsCount=500;
	float reprojectionError=15.0;
	// portion of inliers in matches
	float minInliers = 0.8;
	int flag = CV_ITERATIVE; //CV_ITERATIVE CV_P3P CV_EPNP

	// get 3D-2D registration for all matched frames
	for(auto pair = pairs.begin(); pair != pairs.end(); pair++){


		int i = (pair -> pair_index).first;
		int j = (pair -> pair_index).second;

		std::vector<Point2f> keyPoints1 = pair -> matched_points.first;
		std::vector<Point2f> keyPoints2 = pair -> matched_points.second;
		Depths depths1 = pair-> pair_depths.first;
		Depths depths2 = pair-> pair_depths.second;


		// get depth value of all matched keypoints in image1
		std::vector<Point3f> points3D;
		std::vector<Point2f> valid_keyPoints;
		for(std::size_t k = 0; k < keyPoints1.size(); ++k){

			float x = keyPoints1[k].x;
			float y = keyPoints1[k].y;
			float d = depths1[k];
			// skip invalid depth
			if (d == 0) continue;
			// backproject 3d points
			Mat new_point = d * cameraMatrix.inv() * (Mat_<float>(3,1) << x,y,1.f);
			//Point3f new_point = backproject3D(x,y,d,cameraMatrix);
			valid_keyPoints.push_back(keyPoints2[k]);
			points3D.push_back(Point3f(new_point));
		}

		Mat rvec,tvec,R;
		Mat inliers;
		//NOTE the following command always fails
		solvePnPRansac(points3D,valid_keyPoints,cameraMatrix,distCoeffs,
			 rvec,tvec,
			 useExtrinsicGuess, iterationsCount, reprojectionError, .95, inliers, flag);

		_log("%03d inliers for %04d-%04d pair.", inliers.rows, i, j);

		tvec.convertTo(tvec,CV_32FC1);
		rvec.convertTo(rvec,CV_32FC1);

		Rodrigues(rvec,R);
		pair -> R = R;
		pair -> t = tvec;
		//TODO CheckCoherentRotation(R);
		/*
		std::vector<cv::Point2f> projected3D;
		cv::projectPoints(points3D, rvec, tvec, cameraMatrix, distCoeffs, projected3D);
		for(int i=0;i<projected3D.size();i++) {
			std::cout << i << ". " << valid_keyPoints[i] << std::endl;
			std::cout << i << ". " << projected3D[i] << std::endl;
		}
		*/
	}

	_log("Complete");
}
/**
* 3D-2D registration using Kinect depth points
*/
// bool Pipeline::ransacRegistration(ImagePair& pair)
// {
// 	int i = (pair -> pair_index).first;
// 	int j = (pair -> pair_index).second;

// 	std::vector<Point2f> keyPoints1 = pair -> matched_points.first;
// 	std::vector<Point2f> keyPoints2 = pair -> matched_points.second;
// 	Depths depths1 = pair-> pair_depths.first;
// 	Depths depths2 = pair-> pair_depths.second;

// 	// get depth value of all matched keypoints in image1
// 	std::vector<Point3f> points3D;
// 	std::vector<Point2f> valid_keyPoints;
// 	for(std::size_t k = 0; k < keyPoints1.size(); ++k){

// 		float x = keyPoints1[k].x;
// 		float y = keyPoints1[k].y;
// 		float d = depths1[k];
// 		// skip invalid depth
// 		if (d == 0) continue;
// 		// backproject 3d points
// 		Mat new_point = d * cameraMatrix.inv() * (Mat_<float>(3,1) << x,y,1.f);
// 		// Point3f new_point = backproject3D(x,y,d,cameraMatrix);
// 		valid_keyPoints.push_back(keyPoints2[k]);
// 		points3D.push_back(Point3f(new_point));
// 	}

// 	Mat rvec,tvec,R;
// 	Mat inliers;
// 	//NOTE the following command always fails
// 	// solvePnPRansac(points3D,valid_keyPoints,cameraMatrix,distCoeffs,
// 	// 	rvec,tvec,
// 	// 	useExtrinsicGuess, iterationsCount, reprojectionError,100, noArray(), flag);
// 	solvePnPRansac(points3D,valid_keyPoints,cameraMatrix,distCoeffs,
// 		rvec,tvec);
// 	tvec.convertTo(tvec,CV_32FC1);
// 	rvec.convertTo(rvec,CV_32FC1);

// 	Rodrigues(rvec,R);
// 	pair -> R = R;
// 	pair -> t = tvec;
// 	//TODO CheckCoherentRotation(R);
// 	std::cout << " test" << std::endl<< R <<std::endl;
// 	std::vector<cv::Point2f> projected3D;
// 	cv::projectPoints(points3D, rvec, tvec, cameraMatrix, distCoeffs, projected3D);
// 	for(int i=0;i<projected3D.size();i++) {
// 		std::cout << i << ". " << valid_keyPoints[i] << std::endl;
// 		std::cout << i << ". " << projected3D[i] << std::endl;
// 	}
// }
// bool CheckCoherentRotation(cv::Mat_<float>& R) {

// 	if(fabs(determinant(R))-1.0 > 1e-05) {
// 		std::cerr << "det(R) != +-1.0, this is not a rotation matrix" << std::endl;
// 		return false;
// 	}
// 	return true;
// }
