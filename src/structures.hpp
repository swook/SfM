#pragma once

#include <unordered_map>

#include "opencv2/opencv.hpp"

#include "boost/date_time/posix_time/posix_time.hpp"
namespace pt = boost::posix_time;

/**
 * Information related to RGB-D pair images
 */
struct Image
{
	int 		  index;    // index of frame
	const pt::ptime   time;     // Time taken
	const cv::Mat     rgb;      // 1-channel cv::Mat containing rgb   data
	const cv::Mat     gray;     // 1-channel cv::Mat containing gray  data
	const cv::Mat     dep;      // 1-channel cv::Mat containing depth data
	const std::string rgb_path; // Path to rgb   file
	const std::string dep_path; // Path to depth file

	Image& operator=(Image other)
	{
		index = other.index;
		const_cast<pt::ptime&>  (time    ) = other.time;
		const_cast<cv::Mat&>    (rgb     ) = other.rgb;
		const_cast<cv::Mat&>    (gray    ) = other.gray;
		const_cast<cv::Mat&>    (dep     ) = other.dep;
		const_cast<std::string&>(rgb_path) = other.rgb_path;
		const_cast<std::string&>(dep_path) = other.dep_path;
		return *this;
	}
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
typedef std::vector<float> 		 Depths;

/**
 * Necessary information for each camera frame
 */
struct CamFrame
{
	int             index;      // index of frame
	const KeyPoints key_points; // list of feature points found in this image
	const Depths 		depths;		// list of depth values of features found in this image
	
	CamFrame& operator=(CamFrame other)
	{
		const_cast<int&>      (index)      = other.index;
		const_cast<KeyPoints&>(key_points) = other.key_points;
		return *this;
	}
};

typedef std::vector<CamFrame> CamFrames;

/**
 * Pair of images which have sufficient no. of matching features
 */
typedef std::pair<std::vector<cv::Point2f>, std::vector<cv::Point2f> > KeyPointsPair;
typedef std::pair<std::vector<int>, std::vector<int> >				   MatchIdxPair;
struct ImagePair
{
	const std::pair<int,int>			  pair_index;
	const KeyPointsPair 				  matched_points;
	const MatchIdxPair		  	      	  matched_indices;
	const std::pair<Depths,Depths>		  pair_depths;
	cv::Mat 							  R;
	cv::Mat           					  t;
};
typedef std::vector<ImagePair>   ImagePairs;
typedef std::vector<ImagePair*> pImagePairs;

/**
 * Hash function for unordered_map
 */
	template <class T>
inline void hash_combine(std::size_t & seed, const T & v)
{
	std::hash<T> hasher;
	seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

namespace std
{
	template<typename S, typename T> struct hash<pair<S, T>>
	{
		inline size_t operator()(const pair<S, T> & v) const
		{
			size_t seed = 0;
			::hash_combine(seed, v.first);
			::hash_combine(seed, v.second);
			return seed;
		}
	};
}


/**
 * Unordered map to store idx of triangulated 3d point of (img_c,keypoint_k)
 */
typedef std::unordered_map<std::pair<int,int> ,int> PointMap;

/**
 * PointCluster structure for clusters
 */
typedef std::vector<cv::Point3f> 	PointCluster; 	//from depth map back projected depth points

typedef std::vector<PointCluster> PointClusters;

/**
 * PointCluster structure for point after CoM
 */
typedef std::vector<cv::Point3f> PointCloud;


/**
 * Global Camera Pose
 */
struct CameraPose
{
	cv::Mat R;
	cv::Mat t;
};
typedef std::vector<CameraPose> CameraPoses;

