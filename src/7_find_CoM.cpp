#include "opencv2/core.hpp"
using namespace cv;

#include "Pipeline.hpp"
#include "Associativity.hpp"
#include "util.hpp"

void Pipeline::find_CoM(const PointClusters& pointClusters,Images& images,CamFrames& camFrames,PointMap& pointMap, PointCloud& pointCloud)
{
	Logger _log("Step 7 (global)");
	std::cout<< "cluster number =  " << pointClusters.size() << std::endl;

	for(size_t c=0;c<pointClusters.size();c++){
		cv::Mat mean_;
		reduce(pointClusters[c], mean_, 1, CV_REDUCE_AVG);
		cv::Point3f mean(mean_.at<float>(0,0), mean_.at<float>(0,1),mean_.at<float>(0,2));
		pointCloud[c] = mean;
		// std::cout<< pointClusters[c] << std::endl;
		// std::cout<< pointCloud[c] << std::endl;
	}

	Mat rvec;
	Rodrigues(Mat::eye(3, 3, CV_32F), rvec);
	// debug plot reprojection
	Mat img0 = images[0].rgb;
	for(size_t k = 0; k < camFrames[0].key_points.size();k++)
	{
		if (pointMap.find(std::pair<int,int> (0,k))== pointMap.end()) continue;
		
		circle(img0,camFrames[0].key_points[k].pt,1,CV_RGB(255,0,0),3);
		
		int pIdx = pointMap.find(std::pair<int,int> (0,k)) -> second;
		
		std::vector<cv::Point2f> imagePoints;
		std::vector<cv::Point3f> point3D = {pointCloud[pIdx]} ;
		
		projectPoints(point3D, rvec, Mat::zeros(1, 3,CV_32F), cameraMatrix, noArray(), imagePoints);
		circle(img0,imagePoints[0],1,CV_RGB(0,255,0),3);
	}
	
	imshow("reprojection ",img0);
	waitKey(0);
	_log.tok();
}