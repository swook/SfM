#include "opencv2/core.hpp"
using namespace cv;

#include "Pipeline.hpp"
#include "util.hpp"

void _build_spanning_tree(int i, std::vector<bool> checkedCamera,
	CameraPoses& cameraPoses, const ImagePairs& pairs,
	const Associativity& assocMat);

void Pipeline::build_spanning_tree(CameraPoses& cameraPoses, const ImagePairs& pairs,
	const Associativity& assocMat)
{
	Logger _log("Step 4 (spanning)");

	// Vector to check if camera has global R,t calculated
	std::vector<bool> checkedCamera(assocMat.n);

	// Add first camera
	cameraPoses = CameraPoses(assocMat.n);
	cameraPoses[0] = ((CameraPose) {
		Mat::eye(3, 3, CV_32F),  // R
		Mat::zeros(3, 1, CV_32F) // t
	});
	checkedCamera[0] = true;

	// Build rest of tree
	_build_spanning_tree(0, checkedCamera, cameraPoses, pairs, assocMat);

	_log.tok();
}

void _build_spanning_tree(int i, std::vector<bool> checkedCamera,
	CameraPoses& cameraPoses, const ImagePairs& pairs,
	const Associativity& assocMat)
{
	pImagePairs found_pairs = getAssociatedPairs(i, assocMat);
	ImagePair* pair;
	for (int p = 0; p < found_pairs.size(); p++)
	{
		pair = found_pairs[p];
		int j = pair->pair_index.first != i ?
		        pair->pair_index.first :
		        pair->pair_index.second;
		if (checkedCamera[j]) continue; // Already got R,t

		Mat R, t;
		if (pair->pair_index.first == i) { // If i-j pair
			R = pair->R.inv();
			t = -pair->t;
		} else {                           // If j-i pair
			R = pair->R;
			t = pair->t;
		}

		// Calculate R_j and t_j
		cameraPoses[j].R = cameraPoses[i].R * R;
		cameraPoses[j].t = cameraPoses[i].t + t;
		checkedCamera[j] = true; // Done

		_build_spanning_tree(j, checkedCamera, cameraPoses, pairs, assocMat);
	}
}

