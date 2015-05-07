#pragma once

#include <cmath>
#include <cstdarg>

#include "opencv2/opencv.hpp"

typedef unsigned int uint;

void showImage(const char* title, const Mat& img);
void showImageAndWait(const char* title, const Mat& img);
Point3f backproject3D(const float x,const float y, float depth, const Mat m_cameraMatrix);

/**
 * Custom logger, instantiate with namespace string to prefix messages with.
 * For example:
 *     Logger _log("Load Images");
 */
class Logger {
private:
	const char* name_space;

public:
	Logger(const char* _namespace);
	void operator() (const char* format, ...);
};

