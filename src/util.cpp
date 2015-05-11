#include <cmath>
#include <cstdarg>
#include <assert.h>

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

/**
 * Custom logger, instantiate with namespace string to prefix messages with.
 * For example:
 *     Logger _log("Load Images");
 */
Logger::Logger(const char* _namespace)
	: name_space(_namespace), start(ch::system_clock::now())
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
	const clock_t    end = ch::system_clock::now();
	ch::milliseconds dur = ch::duration_cast<ch::milliseconds>(end - start);
	(*this)("Done in %.2fs.", (float)dur.count() / 1e3);
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
	float r00 = R.at<float>(0,0);
	float r01 = R.at<float>(0,1);
	float r02 = R.at<float>(0,2);
	float r10 = R.at<float>(1,0);
	float r11 = R.at<float>(1,1);
	float r12 = R.at<float>(1,2);
	float r20 = R.at<float>(2,0);
	float r21 = R.at<float>(2,1);
	float r22 = R.at<float>(2,2);

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
	Vec4f q = normalize(Vec4f(q0, q1, q2, q3));
	return q;
}

Mat quat2R(Vec4f& q){

    float sqw = q[0]*q[0];
    float sqx = q[1]*q[1];
    float sqy = q[2]*q[2];
    float sqz = q[3]*q[3];

    // invs (inverse square length) is only required if quaternion is not already normalised
    float invs = 1 / (sqx + sqy + sqz + sqw);
    float m00 = ( sqx - sqy - sqz + sqw)*invs ; // since sqw + sqx + sqy + sqz =1/invs*invs
    float m11 = (-sqx + sqy - sqz + sqw)*invs ;
    float m22 = (-sqx - sqy + sqz + sqw)*invs ;
    
    float tmp1 = q[1]*q[2];
    float tmp2 = q[3]*q[0];
    float m10 = 2.0 * (tmp1 + tmp2)*invs ;
    float m01 = 2.0 * (tmp1 - tmp2)*invs ;
    
    tmp1 = q[1]*q[3];
    tmp2 = q[2]*q[0];
    float m20 = 2.0 * (tmp1 - tmp2)*invs ;
    float m02 = 2.0 * (tmp1 + tmp2)*invs ;
    tmp1 = q[2]*q[3];
    tmp2 = q[1]*q[0];
    float m21 = 2.0 * (tmp1 + tmp2)*invs ;
    float m12 = 2.0 * (tmp1 - tmp2)*invs ;
    Mat R = (Mat_<float>(3,3) << m00, m01, m02, m10, m11, m12, m20, m21, m22);
    return R;
}

bool checkCoherentRotation(cv::Mat& R) {
	
	if(fabs(determinant(R))-1.0 > 1e-05) return false;
	return true;
}

bool checkCoherentQ(Vec4f& q0, Vec4f& q1)
{	
	Vec4f absdelta,reldelta;
	absdiff(q0,q1,absdelta);
	divide(absdelta,q1,reldelta);
	if (reldelta[0]>0.05 ||reldelta[1]>0.05 ||reldelta[2]>0.05 ||reldelta[3]>0.05)
		return false;
	return true;
}

