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
	int iterationsCount=1000;
	float reprojectionError=8.0;
	// portion of inliers in matches
	float minInliers = 0.90;
	float minInliers_check = 0.75;
	int flag = SOLVEPNP_EPNP; //SOLVEPNP_ITERATIVE SOLVEPNP_P3P SOLVEPNP_EPNP SOLVEPNP_DLS SOLVEPNP_UPNP

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
		// std::vector<Point2f> valid_keyPoints_i,valid_keyPoints_j;
		for(std::size_t k = 0; k < keyPoints_i.size(); ++k){

			float x_i = keyPoints_i[k].x;
			float y_i = keyPoints_i[k].y;
			float d_i = depths_i[k];
			float x_j = keyPoints_j[k].x;
			float y_j = keyPoints_j[k].y;
			float d_j = depths_j[k];
			
			// backproject 3d points
			Point3f objPoints_i = backproject3D(x_i,y_i,d_i,cameraMatrix);
			// valid_keyPoints_i.push_back(keyPoints_i[k]);
			points3D_i.push_back(objPoints_i);

			Point3f objPoints_j = backproject3D(x_j,y_j,d_j,cameraMatrix);
			// valid_keyPoints_j.push_back(keyPoints_j[k]);
			points3D_j.push_back(objPoints_j);
		}

		Mat rvec_i,tvec_i,R_i,rvec_j,tvec_j,R_j;
		Mat inliers_i,inliers_j;
		//get R_i and t_i wrt camera j
		solvePnPRansac(points3D_i,keyPoints_j,
			cameraMatrix,noArray(),
			rvec_i,tvec_i,
			useExtrinsicGuess, iterationsCount, reprojectionError, minInliers, inliers_i, flag);
		// get R_j and t_j wrt camera i
		solvePnPRansac(points3D_j,keyPoints_i,
			cameraMatrix,noArray(),
			rvec_j,tvec_j,
			useExtrinsicGuess, iterationsCount, reprojectionError, minInliers, inliers_j, flag);
		
		_log("%03d inliers for %04d-%04d pair.", inliers_i.rows, i, j);

		tvec_i.convertTo(tvec_i,CV_32FC1);
		rvec_i.convertTo(rvec_i,CV_32FC1);
		tvec_j.convertTo(tvec_j,CV_32FC1);
		rvec_j.convertTo(rvec_j,CV_32FC1);
		
		// average R_i and R_j transpose using quaternion
		Mat r,R,tvec;
		r = (rvec_i-rvec_j)/2;
		tvec = (tvec_i - tvec_j)/2;
		Rodrigues(r,R);
		
		// TODO check validity
		if (!checkCoherentRotation(R))
		{
			_log("Invalid R in %04d-%04d, this pair is skipped!", i, j);
			continue;
		}
		Mat _rvev_j = -rvec_j;
		std::cout << "rvec" << std::endl;
		std::cout << _rvev_j << std::endl;
		std::cout << rvec_i << std::endl;
		if (!checkCoherent(rvec_i,_rvev_j))		
		{		
			_log("Invalid q in %04d-%04d, this pair is skipped!", i, j);		
			continue;
		}

		if (inliers_j.rows< (int)(keyPoints_j.size()*minInliers_check) || inliers_i.rows< (int) (keyPoints_i.size()*minInliers_check)){
			_log("Too few inliers in %04d-%04d, this pair is skipped!", i, j);
			continue;	
		} 

		// get rid of outliers
		int inliers_idx_j = 0;
		int inliers_idx_i = 0;

		std::vector<int> new_matched_indices_i;
		std::vector<int> new_matched_indices_j;
		std::vector<cv::Point2f> new_matched_kp_i;
		std::vector<cv::Point2f> new_matched_kp_j;


		while (inliers_idx_i != inliers_i.rows && inliers_idx_j != inliers_j.rows) {

	        if (inliers_i.at<int>(inliers_idx_i,0) < inliers_j.at<int>(inliers_idx_j,0)) {
	            ++inliers_idx_i;
	        } 
	        else
	        {
	            if (!(inliers_j.at<int>(inliers_idx_j,0) < inliers_i.at<int>(inliers_idx_i,0))) {
	                new_matched_indices_i.push_back(pair -> matched_indices.first[inliers_idx_i]);
	                new_matched_indices_j.push_back(pair -> matched_indices.second[inliers_idx_j]);
	                new_matched_kp_i.push_back(pair -> matched_points.first[inliers_idx_i]);
	                new_matched_kp_j.push_back(pair -> matched_points.second[inliers_idx_j]);
        		}
	            ++inliers_idx_j;
        	}
	    }
		pair -> matched_indices.first = new_matched_indices_i;
		pair -> matched_indices.second = new_matched_indices_j;
		pair -> matched_points.first = new_matched_kp_i;
		pair -> matched_points.second = new_matched_kp_j;

		pair -> R = R;
		pair -> t = tvec;
	}

	_log.tok();
}
