#pragma once

#include "opencv2/opencv.hpp"

#include "boost/date_time/posix_time/posix_time.hpp"
namespace pt = boost::posix_time;

/**
 * Information related to RGB-D pair images
 */
struct Image
{
	const pt::ptime   time;     // Time taken
	const cv::Mat     rgb;      // 3-channel cv::Mat containing RGB   data
	const cv::Mat     dep;      // 1-channel cv::Mat containing depth data
	const std::string rgb_path; // Path to rgb   file
	const std::string dep_path; // Path to depth file
};

/**
 * List of Image structs
 */
typedef std::vector<Image> Images;

