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
	const cv::Mat     gray;     // 1-channel cv::Mat containing gray  data
	const cv::Mat     dep;      // 1-channel cv::Mat containing depth data
	const std::string rgb_path; // Path to rgb   file
	const std::string dep_path; // Path to depth file
};

/**
 * List of Image structs
 */
typedef std::vector<Image> Images;

/**
 * List of key points
 */
typedef std::vector<cv::KeyPoint> KeyPoints;

// Descriptor-related helper typedefs
typedef cv::Mat                  Descriptors;
typedef std::vector<Descriptors> DescriptorsVec;


/**
 * Necessary information for each camera frame
 */
struct CamFrame
{
	const KeyPoints key_points; // list of feature points found in this image
	// TODO add field for pose when pose_estimation R and t
};

typedef std::vector<CamFrame> CamFrames;

/**
 * Pair of images which have sufficient no. of matching features
 */
typedef std::pair<KeyPoints, KeyPoints> KeyPointsPair;
struct ImagePair
{
	const std::pair<KeyPoints, KeyPoints> keypoints;
};
typedef std::vector<ImagePair> ImagePairs;

