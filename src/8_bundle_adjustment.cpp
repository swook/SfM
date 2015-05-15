#include "ceres/ceres.h"
#include "ceres/rotation.h"

#include "opencv2/core.hpp"

#include "util.hpp"
#include "Pipeline.hpp"

using namespace cv;

/*
	structure to evaluate residual and jacobian
*/
struct ErrorFunctor {

	// constructor measurements as argument
	ErrorFunctor(const float observed_x, const float observed_y, 
		const float observed_depth,const Mat& cameraMatrix)
	{
		// unproject observed 3D point
		// Point3f backProjected;
		// backProjected = backproject3D(observed_x,observed_y,observed_depth,cameraMatrix);
		// double len = norm(backProjected);
		// localPoint3D_obs_tmp[0] = (double) backProjected.x/len;
		// localPoint3D_obs_tmp[1] = (double) backProjected.y/len;
		// localPoint3D_obs_tmp[2] = (double) backProjected.z/len;
		_observed_x = (double) observed_x;
		_observed_y = (double) observed_y;
		_observed_depth = (double) observed_depth;
		focal_x = cameraMatrix.at<float>(0,0);
		focal_y = cameraMatrix.at<float>(1,1);
		c_x = cameraMatrix.at<float>(0,2);
		c_y = cameraMatrix.at<float>(1,2);
	}

	double _observed_x;
	double _observed_y;
	double _observed_depth;
	double localPoint3D_obs_tmp[3];
	double focal_x;
	double focal_y;
	double c_x;
	double c_y;
	/*
		method to compute residual given rotation vector and translation vector of camera i
		and 3D point j
	*/
    template <typename T>
    bool operator()(const T* const c,
              		const T* const p,
              		T* residuals) const {

  //   	/*
		// angular term
  //   	*/
  //   	// observed 3D point is back projected pixel
  //   	T localPoint3D_obs[3];

  //   	localPoint3D_obs[0] = T(localPoint3D_obs_tmp[0]);
  //   	localPoint3D_obs[1] = T(localPoint3D_obs_tmp[1]);
  //   	localPoint3D_obs[2] = T(localPoint3D_obs_tmp[2]);

  //   	// estimated 3D point is estimated point3D transformed in local camera
  //   	T localPoint3D_est[3];
  //   	T point3D[3];
  //   	ceres::AngleAxisRotatePoint(c,p,point3D);
  //   	localPoint3D_est[0] += c[3]; 
  //   	localPoint3D_est[1] += c[4]; 
  //   	localPoint3D_est[2] += c[5];
  //   	T squaredNorm = localPoint3D_est[0]*localPoint3D_est[0]
  //   		+ localPoint3D_est[1]*localPoint3D_est[1]
  //   		+ localPoint3D_est[2]*localPoint3D_est[2];
  //   	if (squaredNorm>T(0.0)){

  //   		T norm = ceres::sqrt(squaredNorm);

		// 	localPoint3D_est[0] = localPoint3D_est[0]/norm;
  //   		localPoint3D_est[1] = localPoint3D_est[1]/norm;
  //   		localPoint3D_est[2] = localPoint3D_est[2]/norm;
  //   	}else{
  //   		localPoint3D_est
  //   	}
		

  //   	residuals[0] = T(1.0) - 
  //   	(localPoint3D_est[0]*localPoint3D_obs[0] + 
		//  localPoint3D_est[1]*localPoint3D_obs[1] +
		//  localPoint3D_est[2]*localPoint3D_obs[2] );

	/*
	depth term
	*/
	// T depth_est = p[2];
	// T depth_obs = T(_observed_depth);
	// residuals[2] = T(5/depth_est) * (depth_est - depth_obs);

    /*
	reprojection error
	*/

    T _point[3],point[3] ;
    // ceres::AngleAxisRotatePoint(c, p, point);
    // // camera[3,4,5] are the translation.
    // point[0] += c[3]; point[1] += c[4]; point[2] += c[5];

    _point[0] = p[0] + c[3];
    _point[1] = p[1] + c[4];
    _point[2] = p[2] + c[5];

    ceres::AngleAxisRotatePoint(c, _point, point);

    // Compute the center of distortion. The sign change comes from
    // the camera model that Noah Snavely's Bundler assumes, whereby
    // the camera coordinate system has a negative z axis.
    T xp =  point[0] / point[2];
    T yp =  point[1] / point[2];

    // Compute final projected point position.
    T predicted_x = T(focal_x) * xp + T(c_x);
    T predicted_y = T(focal_y) * yp + T(c_y);

    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - T(_observed_x);
    residuals[1] = predicted_y - T(_observed_y);
     	
    return true;
    }

    // Factory to hide the construction of the CostFunction object from
		// the client code.
		static ceres::CostFunction* Create(const float observed_x,
                                  const float observed_y,
                                  const float observed_depth,
                                  const Mat& cameraMatrix) 
		{
 			return (new ceres::AutoDiffCostFunction<ErrorFunctor, 2, 6, 3>(
             new ErrorFunctor(observed_x, observed_y,observed_depth, cameraMatrix)));
		}

};

void Pipeline::bundle_adjustment(
	const PointMap& pointMap,
	const CamFrames& camFrames,
	CameraPoses& poses,
	PointCloud& pointCloud)
{
	Logger _log("Step 8 (BA)");
	// number of parameters per camera and per point
	const int pCamera_num = 6;			// rotation vector 3x1 + translation vector 3x1
	const int pPoint_num = 3;			// point 3x1
	const double huberParam = 1.0;
	const double cauchyParam = 1.0;
	const double function_tolerance = 1e-5;
	const double parameter_tolerance = 1e-8;

	const int camera_num = camFrames.size();
	/*
		set initial parameter
	*/
	double cameras[pCamera_num*camera_num];
	double points[pointCloud.size()*pPoint_num];
	ceres::Problem problem;	
	ceres::LossFunction* loss_function = new ceres::CauchyLoss(cauchyParam);
	for (auto it = pointMap.begin();it != pointMap.end(); it++)
	{
		// parameters associated to 3Dpoint
		int pointIdx = it -> second;
		
		Point3d point3D = Point3_<double>(pointCloud[pointIdx]);

		int pPoint_idx = pointIdx*pPoint_num;
		points[pPoint_idx] = point3D.x; 
		points[pPoint_idx+1] = point3D.y; 
		points[pPoint_idx+2] = point3D.z;	

		// parameters associated to cameras
		int camIdx = (it-> first).first;
		int kpIdx = (it-> first).second;
	
		// camera pose
		Mat r;
		Rodrigues(poses[camIdx].R,r);
		Mat t = poses[camIdx].t;
		// convert to array
		int pCamera_idx = pCamera_num * camIdx;
		for(int i =0; i<3;i++){
			cameras[pCamera_idx+i] = (double) r.at<float>(i,0);
			cameras[pCamera_idx+i+3] = (double) t.at<float>(i,0);
		}

		// observed pixel
		float observed_x = camFrames[camIdx].key_points[kpIdx].pt.x;
		float observed_y = camFrames[camIdx].key_points[kpIdx].pt.y;
		float observed_depth = camFrames[camIdx].depths[kpIdx];
		
		ceres::CostFunction* cost_function =
      		ErrorFunctor::Create(
           		observed_x,observed_y,observed_depth,cameraMatrix);
  		problem.AddResidualBlock(cost_function,loss_function,cameras+pCamera_idx,points+pPoint_idx);
	}
	std::cout << "problem created " << std::endl;
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::SPARSE_SCHUR;
	options.use_inner_iterations = true;
	options.max_num_iterations = 100;
	options.function_tolerance = function_tolerance;
	options.parameter_tolerance = parameter_tolerance;
	options.num_linear_solver_threads = 8;
	options.num_threads = 8;
	options.minimizer_progress_to_stdout = true;
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	std::cout << summary.FullReport() << "\n";

	// write out the result
	for(int pointIdx = 0; pointIdx<pointCloud.size();pointIdx++){
		pointCloud[pointIdx].x = (float) points[pointIdx*pPoint_num];
		pointCloud[pointIdx].y = (float) points[pointIdx*pPoint_num+1];
		pointCloud[pointIdx].z = (float) points[pointIdx*pPoint_num+2];
	}

	for(int camIdx = 0; camIdx<poses.size(); camIdx ++){
		if (poses[camIdx].R.empty()) continue;

		double r_arr[3];
		r_arr[0] = cameras[camIdx*pCamera_num];
		r_arr[1] = cameras[camIdx*pCamera_num + 1];
		r_arr[2] = cameras[camIdx*pCamera_num + 2];
		double t_arr[3];
		t_arr[0] = cameras[camIdx*pCamera_num + 3];
		t_arr[1] = cameras[camIdx*pCamera_num + 4];
		t_arr[2] = cameras[camIdx*pCamera_num + 5];

		Mat r(3, 1, CV_64F, r_arr);
		r.convertTo(r,CV_32F);
		Mat R;
		Rodrigues(r,R);
		poses[camIdx].R = R;
		
		Mat t(3, 1, CV_64F, t_arr);
		t.convertTo(t,CV_32F);
		poses[camIdx].t = t;


	}
}