#include <algorithm>

#include "opencv2/opencv.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
using namespace cv;

#include "Pipeline.hpp"
#include "util.hpp"

void Pipeline::register_camera(ImagePairs& pairs,CamFrames& cam_Frames){
	Logger _log("Step 3 (register)");

	// ---------Parameters for solvePnPRansac----------------//
	const bool  useExtrinsicGuess = false;
	const int   iterationsCount   = 1000;
	const float reprojectionError = 2.5; // Bernhard: should be low, like 3

	const float confidence = 0.999; // Bernhard: should be very high to use more iterations
	const int   minInliers = 30;    // Bernhard: min 12, but 30 is okay for small dataset

	// SOLVEPNP_ITERATIVE SOLVEPNP_P3P SOLVEPNP_EPNP SOLVEPNP_DLS SOLVEPNP_UPNP
	const int flag = SOLVEPNP_EPNP;

	// get 3D-2D registration for all matched frames
#pragma omp parallel for
	for(int p = 0; p < pairs.size(); p++)
	{
		ImagePair* pair = &pairs[p];

		int i = (pair->pair_index).first;
		int j = (pair->pair_index).second;

		std::vector<Point2f> keyPoints_i = pair->matched_points.first;
		std::vector<Point2f> keyPoints_j = pair->matched_points.second;
		Depths depths_i = pair->pair_depths.first;
		Depths depths_j = pair->pair_depths.second;


		// get depth value of all matched keypoints in image1
		std::vector<Point3f> points3D_i, points3D_j;

		for(std::size_t k = 0; k < keyPoints_i.size(); ++k){

			float x_i = keyPoints_i[k].x;
			float y_i = keyPoints_i[k].y;
			float d_i = depths_i[k];

			// backproject 3d points
			Point3f objPoint_i = backproject3D(x_i,y_i,d_i,cameraMatrix);
			points3D_i.push_back(objPoint_i);

			float x_j = keyPoints_j[k].x;
			float y_j = keyPoints_j[k].y;
			float d_j = depths_j[k];

			Point3f objPoint_j = backproject3D(x_j,y_j,d_j,cameraMatrix);
			points3D_j.push_back(objPoint_j);
		}

		Mat rvec_i,tvec_i,R_i,rvec_j,tvec_j,R_j;
		Mat inliers_i,inliers_j;

		// OPENCV:
		// i provides: model coordinate system
		// j provides: camera coordinate system
		//
		// rvec together with tvec, brings points from the model
		// coordinate system to the camera coordinate system.
		// i > j: R_ji

		// US:
		// R_ji * p_i = p_j
		solvePnPRansac(points3D_i,keyPoints_j,
			cameraMatrix,noArray(),
			rvec_i,tvec_i,
			useExtrinsicGuess, iterationsCount, reprojectionError, confidence, inliers_i, flag);

		// get R_ij and t_ij
		solvePnPRansac(points3D_j,keyPoints_i,
			cameraMatrix,noArray(),
			rvec_j,tvec_j,
			useExtrinsicGuess, iterationsCount, reprojectionError, confidence, inliers_j, flag);

		// Bernhard: Use absolute threshold for inliers number
		if (inliers_i.rows < minInliers || inliers_j.rows < minInliers) {
			continue;
		}

		_log("%03d/%03d inliers for %04d-%04d pair.", inliers_i.rows,
			inliers_j.rows, i, j);

		// Convert rvec and tvec to floats
		tvec_i.convertTo(tvec_i,CV_32FC1);
		rvec_i.convertTo(rvec_i,CV_32FC1);
		tvec_j.convertTo(tvec_j,CV_32FC1);
		rvec_j.convertTo(rvec_j,CV_32FC1);

		// Average R_i & R_j and t_i & t_j
		Mat rvec, tvec, R;
		rvec = (rvec_i - rvec_j) / 2.f;
		tvec = (tvec_i - tvec_j) / 2.f;
		Rodrigues(rvec, R);

		// Bernhard: Shouldn't need, Rodrigues should do this
		// check validity
		//if (!checkCoherentRotation(R))
		//{
		//	_log("Invalid R in %04d-%04d, this pair is skipped!", i, j);
		//	continue;
		//}

		// Bernhard: Should be done, good work Yifan
		if (!checkCoherent(rvec_i,rvec_j))
		{
			_log("Invalid r in %04d-%04d, this pair is skipped!", i, j);
			std::cout << rvec_i << std::endl;
			std::cout << rvec_j << std::endl;
			continue;
		}

		// Store R, t
		pair->R = R;
		pair->t = tvec;

		// get rid of outliers
		std::vector<int> new_matched_indices_i;
		std::vector<int> new_matched_indices_j;
		std::vector<cv::Point2f> new_matched_kp_i;
		std::vector<cv::Point2f> new_matched_kp_j;
		Depths new_depths_i;
		Depths new_depths_j;

		// Get list of common inlier indices
		std::vector<int> inliers(max(inliers_i.rows, inliers_j.rows));
		auto it = std::set_intersection(inliers_i.begin<int>(), inliers_i.end<int>(),
			inliers_j.begin<int>(), inliers_j.end<int>(),
			inliers.begin());
		inliers.resize(it - inliers.begin());


		// Go through all existing keypoint pairs
		for (int k = 0; k < pair->matched_indices.first.size(); k++)
		{
			int keyIdx_i = pair->matched_indices.first[k],
			    keyIdx_j = pair->matched_indices.second[k];

			// If both keypoints are inliers
			if (std::find(inliers.begin(), inliers.end(), keyIdx_i) != inliers.end() &&
			    std::find(inliers.begin(), inliers.end(), keyIdx_j) != inliers.end())
			{
				new_matched_indices_i.push_back(keyIdx_i);
				new_matched_indices_j.push_back(keyIdx_j);
				new_matched_kp_i.push_back(pair->matched_points.first[k]);
				new_matched_kp_j.push_back(pair->matched_points.second[k]);
				new_depths_i.push_back(pair->pair_depths.first[k]);
				new_depths_j.push_back(pair->pair_depths.second[k]);
			}

		}

		// Keep only inlier keypoints
		pair->matched_indices.first  = new_matched_indices_i;
		pair->matched_indices.second = new_matched_indices_j;
		pair->matched_points.first   = new_matched_kp_i;
		pair->matched_points.second  = new_matched_kp_j;
		pair->pair_depths.first      = new_depths_i;
		pair->pair_depths.second     = new_depths_j;
	}

	_log.tok();
}
