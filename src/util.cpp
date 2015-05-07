#include <cmath>
#include <cstdarg>
#include <assert.h>
#include "opencv2/opencv.hpp"
using namespace cv;

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

/**
 * Custom logger, instantiate with namespace string to prefix messages with.
 * For example:
 *     Logger _log("Load Images");
 */
Logger::Logger(const char* _namespace)
{
	name_space = _namespace;
}

void Logger::operator() (const char* format, ...)
{
	va_list args;
	va_start(args, format);

	printf("%s>\t", name_space);
	vprintf(format, args);
	printf("\n");

	va_end(args);
}
/*
 * convert rotation matrix to quaternion
*/
Vec4f R2Quaternion(Mat& R)
{
	assert(R.rows == 3 && R.cols == 3);
	// typedef typename DataType<float>::work_type _wTp;
	if (R.type() != CV_32F) R.convertTo(R, CV_32F);;
	float q0,q1,q2,q3;
	float r00 = R.at<float>(1,1);
	float r01 = R.at<float>(1,2);
	float r02 = R.at<float>(1,3);
	float r10 = R.at<float>(2,1);
	float r11 = R.at<float>(2,2);
	float r12 = R.at<float>(2,3);
	float r20 = R.at<float>(3,1);
	float r21 = R.at<float>(3,2);
	float r22 = R.at<float>(3,3);

	float trace = r00 + r11 + r22;
	if( trace > 0 ){
		float s = 0.5 / sqrt(trace+ 1.0);
    	q0 = 0.25 / s;
    	q1 = ( r21 - r12 ) * s;
    	q2 = ( r02 - r20 ) * s;
    	q3 = ( r10 - r01 ) * s;
	} else {
    	if ( r00 > r11 && r00 > r22 ) 
    	{
      	float s = 2.0 * sqrt( 1.0 + r00 - r11 - r22);
      	q0 = (r21 - r12 ) / s;
      	q1 = 0.25 * s;
      	q2 = (r01 + r10 ) / s;
      	q3 = (r02 + r20 ) / s;
    	} else if (r11 > r22) {
	    float s = 2.0 * sqrt( 1.0 + r11 - r00 - r22);
	    q0 = (r02 - r20 ) / s;
	    q1 = (r01 + r10 ) / s;
	    q2 = 0.25 * s;
	    q3 = (r12 + r21 ) / s;
    	} else {
      	float s = 2.0 * sqrt( 1.0 + r22 - r00 - r11 );
      	q0 = (r10 - r01 ) / s;
	    q1 = (r02 + r20 ) / s;
	    q2 = (r12 + r21 ) / s;
	    q3 = 0.25 * s;
    	}
	}
	Vec4f q = normalize(Vec<float,4>::Vec(q0, q1, q2, q3));
}


