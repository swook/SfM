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

		std::vector<Point2f> keyPoints_i = pair -> matched_points.first;
		std::vector<Point2f> keyPoints_j = pair -> matched_points.second;
		Depths depths_i = pair-> pair_depths.first;
		Depths depths_j = pair-> pair_depths.second;


		// get depth value of all matched keypoints in image1
		std::vector<Point3f> points3D_i, points3D_j;
		std::vector<Point2f> valid_keyPoints_i,valid_keyPoints_j;
		for(std::size_t k = 0; k < keyPoints_i.size(); ++k){

			float x_i = keyPoints_i[k].x;
			float y_i = keyPoints_i[k].y;
			float d_i = depths_i[k];
			float x_j = keyPoints_j[k].x;
			float y_j = keyPoints_j[k].y;
			float d_j = depths_j[k];
			
			// backproject 3d points
			Point3f objPoints_i = backproject3D(x_i,y_i,d_i,cameraMatrix);
			valid_keyPoints_i.push_back(keyPoints_i[k]);
			points3D_i.push_back(objPoints_i);

			Point3f objPoints_j = backproject3D(x_j,y_j,d_j,cameraMatrix);
			valid_keyPoints_j.push_back(keyPoints_j[k]);
			points3D_j.push_back(objPoints_j);
		}

		Mat rvec_i,tvec_i,R_i,rvec_j,tvec_j,R_j;
		Mat inliers,inliers_j;
		//get R_i and t_i wrt camera j
		solvePnPRansac(points3D_i,valid_keyPoints_j,
			cameraMatrix,noArray(),
			rvec_i,tvec_i,
			useExtrinsicGuess, iterationsCount, reprojectionError, .95, inliers, flag);
		// get R_j and t_j wrt camera i
		solvePnPRansac(points3D_j,valid_keyPoints_i,
			cameraMatrix,noArray(),
			rvec_j,tvec_j,
			useExtrinsicGuess, iterationsCount, reprojectionError, .95, inliers_j, flag);
		
		_log("%03d inliers for %04d-%04d pair.", inliers.rows, i, j);

		tvec_i.convertTo(tvec_i,CV_32FC1);
		rvec_i.convertTo(rvec_i,CV_32FC1);
		tvec_j.convertTo(tvec_j,CV_32FC1);
		rvec_j.convertTo(rvec_j,CV_32FC1);
		
		// average R_i and R_j transpose using quaternion
		Rodrigues(rvec_i,R_i);
		Rodrigues(rvec_j,R_j);
		Mat R_j_t = R_j.t();
		Vec4f q_i = R2Quaternion(R_i);
		Vec4f q_j = R2Quaternion(R_j_t);

		Vec4f q_avg = normalize(q_j + q_i);
		Mat R = quat2R(q_avg);

		pair -> R = R;

		// average t_i and t_j
		Mat tvec = (tvec_i - tvec_j)/2;
		pair -> t = tvec;
		
		std::cout<<"tvec "<<std::endl<<tvec<<std::endl;
		std::cout<<"R "<<std::endl<<R<<std::endl;
	}

	_log.tok();
}
