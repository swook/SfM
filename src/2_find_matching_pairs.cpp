#include "opencv2/opencv.hpp"
#include "opencv2/features2d.hpp"
using namespace cv;

#include "Pipeline.hpp"
#include "util.hpp"

void Pipeline::find_matching_pairs(
	const Images&         images,
	const CamFrames&      camframes,
	const DescriptorsVec& descriptors_vec,
	      ImagePairs&     pairs
)
{
	Logger _log("Step 2 (matching)");

	const int N = descriptors_vec.size();

	// Initialise descriptors matcher
	FlannBasedMatcher flann;
	BFMatcher bfmatcher(cv::NORM_L2, true);

	// Get matches
	std::vector<DMatch> matches;
	Descriptors descriptors_i;
	Descriptors descriptors_j;

	// Match between all image pairs possible
	for (int i = 0; i < N; i++)
	{
		for (int j = i + 1; j < N; j++)
		{
			// Get descriptors from image i and image j
			descriptors_i = descriptors_vec[i];
			descriptors_j = descriptors_vec[j];

			// Must convert types to pass on to FLANN matcher
			descriptors_i.convertTo(descriptors_i, CV_32F);
			descriptors_j.convertTo(descriptors_j, CV_32F);

			/* Match descriptors
			 Output is distance between descriptors */
			flann.match(descriptors_i, descriptors_j, matches);
			// brutal force matcher
			//bfmatcher.match( descriptors_i, descriptors_j, matches );

			//_log("Input: (%d: %d rows, %d: %d rows)\t Output: %d rows",
			//	i, descriptors_i.rows,
			//	j, descriptors_j.rows,
			//	matches.size());

			// Our parameters
			//float max_dist    = 100; // SIFT
			float max_dist    = 480; // BRISK
			int   min_matches = 20;

			// Only pick good matches
			std::vector<DMatch> good_matches;
			for (int m = 0; m < matches.size(); m++)
				if (matches[m].distance < max_dist)
					good_matches.push_back(matches[m]);

			if (good_matches.size() < min_matches) continue;
			_log("Got %03d good matches for %04d-%04d.", good_matches.size(), i, j);

			// only store matched keypoints and their depths
			std::vector<Point2f> matched_keypoints_i, matched_keypoints_j;
			Depths depth_values_i, depth_values_j;
			for (auto it = good_matches.begin(); it != good_matches.end(); it++){
				// only save pt, drop other keypoint members
				Point2f point_i = camframes[i].key_points[it -> queryIdx].pt;
				Point2f point_j = camframes[j].key_points[it -> trainIdx].pt;
				matched_keypoints_i.push_back(point_i);
				matched_keypoints_j.push_back(point_j);
				// save depth of keypoints
				float d_i = images[i].dep.at<float>((int)point_i.y,(int)point_i.x);
				float d_j = images[j].dep.at<float>((int)point_j.y,(int)point_j.x);

				depth_values_i.push_back(d_i);
				depth_values_j.push_back(d_j);
			}

			// Add to pairs structure
			pairs.push_back((ImagePair) {
				std::pair<int,int> (i,j),
				KeyPointsPair(
					matched_keypoints_i,
					matched_keypoints_j
				),
				std::pair<Depths,Depths> (depth_values_i,depth_values_j)
			});

			// add matches to match_map
			// match_map.insert(make_pair(i,j),good_matches);

			// Draw matches
			// NOTE: remove continue; to see images
			continue;
			Mat out;
			drawMatches(images[i].rgb, camframes[i].key_points,
			            images[j].rgb, camframes[j].key_points,
				    good_matches, out,
				    Scalar::all(-1),Scalar::all(-1),std::vector<char>(),DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
			char title[64];
			sprintf(title, "Match %d-%d", i, j);
			showImageAndWait("Match results", out);

		}
	}

	_log.tok();
}

