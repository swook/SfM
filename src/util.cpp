#include <cmath>
#include <cstdarg>

#include "opencv2/opencv.hpp"
using namespace cv;

#include "structures.hpp"
#include "util.hpp"

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

pImagePairs getAssociatedPairs(const int i, const Associativity assocMat)
{
	pImagePairs pairs;
	int n = assocMat.n;

	ImagePair* pair = NULL;
	for (int j = 0; j < n; j++)
	{
		if (i == j) continue;
		pair = assocMat(i, j);
		if (pair != NULL) pairs.push_back(pair);
	}
	return pairs;
}

/**
 * Custom logger, instantiate with namespace string to prefix messages with.
 * For example:
 *     Logger _log("Load Images");
 */
Logger::Logger(const char* _namespace)
	: name_space(_namespace), start(clock())
{}

void Logger::operator() (const char* format, ...)
{
	va_list args;
	va_start(args, format);

	printf("%s>\t", name_space);
	vprintf(format, args);
	printf("\n");

	va_end(args);
}

void Logger::tok()
{
	(*this)("Done in %.2fs.", (clock() - start) / (float) CLOCKS_PER_SEC);
}
