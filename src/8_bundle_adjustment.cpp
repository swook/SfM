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
	:
	observed_x((double) observed_x), 
	observed_y((double) observed_y), 
	observed_depth((double) observed_depth){
		cameraMatrix.convertTo(inv_cameraMatrix,CV_64F);
		inv_cameraMatrix = inv_cameraMatrix.inv();
	}

	double observed_x;
	double observed_y;
	double observed_depth;
	Mat inv_cameraMatrix;
	/*
		method to compute residual given rotation vector and translation vector of camera i
		and 3D point j
	*/
    template <typename T>
    bool operator()(const T* const r,
    				const T* const t,
              		const T* const p,
              		T* residuals) const {

    	/*
		angular term
    	*/
    	// observed 3D point is back projected pixel
    	T localPoint3D_obs[3];
    	double c00 = inv_cameraMatrix.at<double>(0,0);
    	double c01 = inv_cameraMatrix.at<double>(0,1);
    	double c02 = inv_cameraMatrix.at<double>(0,2);
    	double c10 = inv_cameraMatrix.at<double>(1,0);
    	double c11 = inv_cameraMatrix.at<double>(1,1);
    	double c12 = inv_cameraMatrix.at<double>(1,2);
    	double c20 = inv_cameraMatrix.at<double>(2,0);
    	double c21 = inv_cameraMatrix.at<double>(2,1);
    	double c22 = inv_cameraMatrix.at<double>(2,2);

    	double tmp0 = c00 * observed_x + c01 * observed_y + c02*1.0;
    	double tmp1 = c10 * observed_x + c11 * observed_y + c12*1.0;
    	double tmp2 = c20 * observed_x + c21 * observed_y + c22*1.0;

    	
    	double len = sqrt(tmp0*tmp0 + tmp1*tmp1 + tmp2*tmp2);

    	localPoint3D_obs[0] = T(tmp0/len);
    	localPoint3D_obs[1] = T(tmp1/len);
    	localPoint3D_obs[2] = T(tmp2/len);

    	// estimated 3D point is estimated point3D transformed in local camera
    	T localPoint3D_est[3];
    	T point3D[3];
    	ceres::AngleAxisRotatePoint(r,p,point3D);
    	localPoint3D_est[0] += t[0]; 
    	localPoint3D_est[1] += t[1]; 
    	localPoint3D_est[2] += t[2];
    	
		T norm = sqrt(localPoint3D_est[0]*localPoint3D_est[0]
    		+ localPoint3D_est[1]*localPoint3D_est[1]
    		+ localPoint3D_est[2]*localPoint3D_est[2]);

		localPoint3D_est[0] = localPoint3D_est[0]/norm;
    	localPoint3D_est[1] = localPoint3D_est[1]/norm;
    	localPoint3D_est[2] = localPoint3D_est[2]/norm;

    	residuals[0] = T(1.0) - 
    	(localPoint3D_est[0]*localPoint3D_obs[0] + 
		 localPoint3D_est[1]*localPoint3D_obs[1] +
		 localPoint3D_est[2]*localPoint3D_obs[2] );

    	/*
		depth term
    	*/
    	T depth_est = sqrt((p[0] - t[0])*(p[0] - t[0])
    	 + (p[1] - t[1])*(p[1] - t[1])
    	 + (p[2] - t[2])*(p[2] - t[2]));
		T depth_obs = T(observed_depth);
    	residuals[1] = depth_est - depth_obs;
     	
     return true;
    }

    // Factory to hide the construction of the CostFunction object from
		// the client code.
		static ceres::CostFunction* Create(const double observed_x,
                                  const double observed_y,
                                  const double observed_depth,
                                  const Mat& cameraMatrix) 
		{
 			return (new ceres::AutoDiffCostFunction<ErrorFunctor, 2, 3, 3, 3>(
             new ErrorFunctor(observed_x, observed_y,observed_depth,cameraMatrix)));
		}

};

void Pipeline::bundle_adjustment(
	const PointMap& pointMap,
	const CameraPoses& poses,
	const CamFrames& camFrames,
	PointCloud pointCloud)
{
	Logger _log("Step 8 (BA)");
	// number of parameters per camera and per point
	const int pCamera_num = 6;			// rotation vector 3x1 + translation vector 3x1
	const int pPoint_num = 3;			// point 3x1
	const double huberParam = 16.0;
	const int camera_num = poses.size();
	/*
		set initial parameter
	*/
	double cameras[pCamera_num*camera_num];
	double points[pointCloud.size()];

	ceres::Problem problem;	
	// ceres::LossFunction* loss_function = new ceres::HuberLoss(huberParam);
	for (auto it = pointMap.begin();it != pointMap.end(); it++)
	{
		// parameters associated to 3Dpoint
		int pointIdx = it -> second;
		Point3d point3D = Point3_<double>(pointCloud[pointIdx]);
		
		int pPoint_idx = pointIdx*camera_num;
		points[pPoint_idx] = point3D.x; 		pPoint_idx++;
		points[pPoint_idx] = point3D.y; 		pPoint_idx++;
		points[pPoint_idx] = point3D.z;			pPoint_idx++;

		// parameters associated to cameras
		int camIdx = (it-> first).first;
		int kpIdx = (it-> first).second;

		// camera pose
		Mat r;
		Rodrigues(poses[camIdx].R,r);
		Mat t = poses[camIdx].t;
		// convert to array
		std::cout << t << std::endl;
		std::cout << r << std::endl;
		int pCamera_idx = pCamera_num * camera_num;
		for(int i =0; i<3;i++){
			cameras[pCamera_idx+i] = (double) t.at<float>(i,0);
			cameras[pCamera_idx+i+3] = (double) r.at<float>(i,0);
		}

		// observed pixel
		float observed_x = camFrames[camIdx].key_points[kpIdx].pt.x;
		float observed_y = camFrames[camIdx].key_points[kpIdx].pt.y;
		float observed_depth = camFrames[camIdx].depths[kpIdx];
		ceres::CostFunction* cost_function =
      		ErrorFunctor::Create(
           		observed_x,observed_y,observed_depth,cameraMatrix);
  		// problem.AddResidualBlock(cost_function,NULL,cameras+pCamera_idx,points+pPoint_idx);
  		break;
	}
	
}