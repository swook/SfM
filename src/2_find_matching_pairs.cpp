#include "opencv2/opencv.hpp"
#include "opencv2/features2d.hpp"
using namespace cv;

#include "Pipeline.hpp"
#include "util.hpp"

Logger _log("Step 2 (match)");

void Pipeline::find_matching_pairs(
	const Images&         images,
	const CamFrames&      camframes,
	const DescriptorsVec& descriptors_vec,
	      ImagePairs&     pairs
)
{
	const int N = descriptors_vec.size();

	// Initialise descriptors matcher
	FlannBasedMatcher flann;

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

			// Match descriptors
			// Output is distance between descriptors
			flann.match(descriptors_i, descriptors_j, matches);
			//_log("Input: (%d: %d rows, %d: %d rows)\t Output: %d rows",
			//	i, descriptors_i.rows,
			//	j, descriptors_j.rows,
			//	matches.size());

			// Our parameters
			float max_dist    = 90;
			int   min_matches = 15;

			// Only pick good matches
			std::vector<DMatch> good_matches;
			for (int m = 0; m < matches.size(); m++)
				if (matches[m].distance < max_dist)
					good_matches.push_back(matches[m]);

			if (good_matches.size() < min_matches) continue;

			// Add to pairs structure
			pairs.push_back((ImagePair) {
				KeyPointsPair(
					camframes[i].key_points,
					camframes[j].key_points
				)
			});

			// Draw matches
			// NOTE: remove continue; to see images
			//continue;
			Mat out;
			drawMatches(images[i].rgb, camframes[i].key_points,
			            images[j].rgb, camframes[j].key_points,
				    good_matches, out);
			char title[64];
			sprintf(title, "Match %d-%d", i, j);
			showImageAndWait("Match results", out);

		}
	}

}

