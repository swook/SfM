#include "ceres/ceres.h"
#include "opencv2/core.hpp"

#include "util.hpp"
#include "Pipeline.hpp"

using namespace ceres;
using namespace cv;

// void Pipeline::bundle_adjustment(PointMap& pointMap,CamFrames& camFrames,PointCloud pointCloud){
// 	/*
// 		structure to evaluate residual and jacobian
// 	*/
// 	struct ErrorFunctor {

// 		// constructor measurements as argument
// 		ErrorFunctor(float _observed_x, float _observed_y, float _measured_depth)
// 		: observed_x(_observed_x), observed_y(_observed_y), measured_depth(_measured_depth){};


// 		float observed_x;
// 		float observed_y;
// 		/*
// 			method to compute residual given rotation vector and translation vector of camera i
// 			and 3D point j
// 		*/
// 	    template <typename T>
// 	    bool operator()(const T* const r,
// 	    				const T* const t,
//                   		const T* const p,
//                   		T* residuals) const {

// 	    	/*
// 	    		angular term
// 	    	*/
// 	    	Mat point3D(3, 1, CV_32F, p);
// 	    	Mat t_vect(3, 1, CV_32F, t);
// 	    	Mat r_vect(3, 1, CV_32F, r);
// 			Mat R;
// 	    	Rodrigues(r_vect,R);
	    	
// 	    	// observed 3D point is back projected pixel
// 	    	Mat localPoint3D_obs = normalize(Mat(backproject3D(observed_x,observed_y,1,cameraMatrix)));
// 	    	// estimated 3D point is estimated point3D transformed in local camera
// 	    	Mat localPoint3D_est = normalize(R * (point3D) + t_vect);
// 	    	residuals[0] = 1.f - (float) localPoint3D_obs.dot(localPoint3D_est);

// 	    	/*
// 	    		depth term
// 	    	*/
//     		float depth_est = (float) norm(point3D - t_vect);
//     		float depth_obs = (float) measured_depth;
// 	    	residuals[1] = depth_est - depth_obs;
	     	
// 	     return true;
// 	    }

// 	    // Factory to hide the construction of the CostFunction object from
//    		// the client code.
//    		static ceres::CostFunction* Create(const double observed_x,
//                                       const double observed_y) {
//      	return (new ceres::AutoDiffCostFunction<ErrorFunctor, 2, 6, 3>(
//                  new ErrorFunctor(observed_x, observed_y)));
//    		}

// 	};

// 	/*
// 		set initial parameter
// 	*/
// 	int parameter_num = pointCloud.size()*3 + 6*
// 	init_x = new double[];
// 	for (auto it = pointMap.begin();it != pointMap.end(); it++)
// 	{
// 		Point3f = pointCloud[it -> second];

// 	}
// 	Problem problem;

// }