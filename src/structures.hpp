#pragma once

#include "opencv2/opencv.hpp"
#include <unordered_map>
#include "boost/functional/hash_fwd.hpp"

#include "boost/date_time/posix_time/posix_time.hpp"
namespace pt = boost::posix_time;

/**
 * Information related to RGB-D pair images
 */
struct Image
{
	int 		  index;	// index of frame
	const pt::ptime   time;     // Time taken
	const cv::Mat     rgb;      // 1-channel cv::Mat containing rgb   data
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
typedef std::vector<float> 		 Depths;

/**
 * Necessary information for each camera frame
 */
struct CamFrame
{
	const int 			index;		// index of frame
	const KeyPoints 	key_points; // list of feature points found in this image
	const Depths		depths;		// list of depths of feature points
	std::vector<int> 	matched_list; // list of camframe indices that are matched to this image

	// TODO add field for pose when pose_estimation R and t (in global frame)
};

typedef std::vector<CamFrame> CamFrames;

/**
 * Pair of images which have sufficient no. of matching features
 */
typedef std::pair<std::vector<cv::Point2f>, std::vector<cv::Point2f> > KeyPointsPair;
struct ImagePair
{
	const std::pair<int,int>			  pair_index;
	const KeyPointsPair 				  matched_points;
	const std::pair<Depths,Depths>		  pair_depths;
	cv::Mat 							  R;
	cv::Mat           					  t;
};
typedef std::vector<ImagePair>   ImagePairs;
typedef std::vector<ImagePair*> pImagePairs;


/*
 Hash function for unordered_map
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

template <>
class std::equal_to<std::pair<int,int> >
{
  public:
     bool operator()(const std::pair<int,int>& a, const std::pair<int,int>& b) const
     {
        return a.first == a.first && a.second == b.second;
     }
};
/**
 * Unordered map to store matches of image pair (idx_i, idx_j)
 */
typedef std::unordered_map<std::pair<int,int> ,std::vector<cv::DMatch> > MatchMap;
/**
 * Unordered map to store idx of triangulated 3d point of (img_c,keypoint_k)
 */
typedef std::unordered_map<std::pair<int,int> ,int> PointMap;

/**
 * CloudPoint structure
 */
 struct CloudPoint
 {
 	cv::Point3f					trangulated;
 	std::vector<cv::Point3f> 	cluster; 	//from depth map back projected depth points
 };
 typedef std::vector<CloudPoint> CloudPoints;


/**
 * Associativity of cameras (matching features, essentially camera pairs)
 */
typedef std::pair<int, int> PairIndex;
class Associativity
{
public:
	std::map<PairIndex, ImagePair*> _map;
	const int n;
	Associativity(int _n) : n(_n) {}
	ImagePair*& operator()(const int i, const int j)
	{
		return _map[PairIndex(i, j)];
	}

	ImagePair* operator()(const int i, const int j) const
	{
		auto found = _map.find(PairIndex(i, j));
		if (found != _map.end()) return found->second;
		else                     return NULL;
	}
};


/**
 * Global Camera Pose
 */
struct CameraPose
{
	cv::Mat R;
	cv::Mat t;
};
typedef std::vector<CameraPose> CameraPoses;

