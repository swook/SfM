#include <utility>

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

	// Our parameters
	const float max_ratio   = 0.8; // larger: more possible mismatches
	const int   min_matches = 20;

	// Match between all image pairs possible
#pragma omp parallel for
	for (int j = 0; j < N; j++)
	{
		// Initialise descriptors matcher
		// Enable cross-checking
		BFMatcher bfmatcher(NORM_L2,true);

		// Get descriptors of j-th image
		Descriptors descriptors_j = descriptors_vec[j];
		descriptors_j.convertTo(descriptors_j, CV_32F);

		Descriptors descriptors_i;

		for (int i = j + 1; i < N; i++)
		{
			// std::vector<std::vector<DMatch>> matches;
			std::vector<DMatch> matches;
			// Get descriptors from image i
			descriptors_i = descriptors_vec[i];

			// Must convert types to pass on to FLANN matcher
			descriptors_i.convertTo(descriptors_i, CV_32F);

			/* Match descriptors
			 Output is distance between descriptors */
			// brute force matcher
			bfmatcher.match(descriptors_i, descriptors_j, matches);

			//_log("Input: (%d: %d rows, %d: %d rows)\t Output: %d rows",
			//	i, descriptors_i.rows,
			//	j, descriptors_j.rows,
			//	matches.size());

			// Stop if not enough matches
			if (matches.size() < min_matches) continue;
#pragma omp critical
			_log("Got %03d matches for %04d-%04d.", matches.size(), i, j);

			// only store matched keypoints and their depths
			std::vector<Point2f> matched_keypoints_i, matched_keypoints_j;
			std::vector<int> matched_indices_i,matched_indices_j;
			Depths depth_values_i, depth_values_j;
			for (auto it = matches.begin(); it != matches.end(); it++){

				assert(camframes[i].key_points.size() > it->queryIdx);
				assert(camframes[j].key_points.size() > it->trainIdx);

				// only save pt, drop other keypoint members
				Point2f point_i = camframes[i].key_points[it -> queryIdx].pt;
				Point2f point_j = camframes[j].key_points[it -> trainIdx].pt;

				matched_keypoints_i.push_back(point_i);
				matched_keypoints_j.push_back(point_j);

				matched_indices_i.push_back(it -> queryIdx);
				matched_indices_j.push_back(it -> trainIdx);

				// save depth of keypoints
				float d_i = camframes[i].depths[it -> queryIdx];
				float d_j = camframes[j].depths[it -> trainIdx];

				depth_values_i.push_back(d_i);
				depth_values_j.push_back(d_j);
			}

			// Add to pairs structure
#pragma omp critical
			pairs.push_back((ImagePair) {
				std::pair<int,int> (i,j),
				KeyPointsPair(
					matched_keypoints_i,
					matched_keypoints_j
					),
				MatchIdxPair(
					matched_indices_i,
					matched_indices_j
					),
				std::pair<Depths,Depths> (depth_values_i,depth_values_j)
			});

			// add matches to match_map
			// match_map.insert(make_pair(i,j), matches);

			// Draw matches
			// NOTE: remove continue; to see images
			continue;
//#pragma omp critical
			{
				std::cout << camframes[i].key_points.size() << "\t"
				        << camframes[j].key_points.size() << "\t"
					<< matches.size() << std::endl;
				Mat out;
				drawMatches(images[i].rgb, camframes[i].key_points,
					    images[j].rgb, camframes[j].key_points,
					    matches, out,
					    Scalar::all(-1),
					    Scalar::all(-1),
					    std::vector<char>(),
					    DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
				char title[64];
				sprintf(title, "Match %d-%d", i, j);
				showImageAndWait("Match results", out);
			}

		}
	}

	_log.tok();
}

