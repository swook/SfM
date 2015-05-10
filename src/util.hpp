#pragma once

#include <cmath>
#include <cstdarg>
#include <ctime>

#include "opencv2/opencv.hpp"

typedef unsigned int uint;

void showImage(const char* title, const cv::Mat& img);
void showImageAndWait(const char* title, const cv::Mat& img);
cv::Point3f backproject3D(const float x,const float y, float depth, const cv::Mat m_cameraMatrix);
cv::Vec4f R2Quaternion(cv::Mat& R);
cv::Mat quat2R(cv::Vec4f& q);
// eigenvalue of rotation matrix should be one (or minus one);
bool checkCoherentRotation(cv::Mat& R);	
// two quaternion shouldn't varie too much
bool checkCoherentQ(cv::Vec4f& q0, cv::Vec4f& q1);

/**
 * Custom logger, instantiate with namespace string to prefix messages with.
 * For example:
 *     Logger _log("Load Images");
 */
class Logger {
private:
	const char* name_space;
	const clock_t  start;

public:
	Logger(const char* _namespace);
	void operator() (const char* format, ...);
	void tok();
};

