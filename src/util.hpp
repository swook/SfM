#pragma once

#include <cmath>
#include <cstdarg>

#include "opencv2/opencv.hpp"
using namespace cv;

typedef unsigned int uint;

void showImage(const char* title, const Mat& img)
{
	std::cout << "\nShowing image: \"" << title << "\"." << std::endl;
	namedWindow(title, CV_WINDOW_NORMAL);
	imshow(title, img);
}

void showImageAndWait(const char* title, const Mat& img)
{
	showImage(title, img);
	std::cout << "Press any key to continue..." << std::endl;
	waitKey(0);
}

Point3f backproject3D(const float x,const float y, float depth, const Mat m_cameraMatrix)
{
	Mat new_point = depth * (Mat_<float>(1,3) << x,y,1) * m_cameraMatrix.inv().t();
	return Point3f(new_point);
}

/**
 * Custom logger, instantiate with namespace string to prefix messages with.
 * For example:
 *     Logger _log("Load Images");
 */
class Logger {
private:
	const char* name_space;

public:
	Logger(const char* _namespace)
	{
		name_space = _namespace;
	}

	void operator() (const char* format, ...)
	{
		va_list args;
		va_start(args, format);

		printf("%s> ", name_space);
		vprintf(format, args);
		printf("\n");

		va_end(args);
	}
};

