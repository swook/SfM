#include "opencv2/core.hpp"
using namespace cv;

#include "Pipeline.hpp"
#include "Associativity.hpp"
#include "util.hpp"

void Pipeline::glo_cam_poses(Images& images, CameraPoses& cameraPoses, const ImagePairs& pairs,
	Associativity& tree)
{
	Logger _log("Step 5 (global)");

	// Add I for R,t of 0th camera
	// (Reference for "global" coordinate frame)
	cameraPoses.resize(tree.n);
	cameraPoses[0].R = Mat::eye(3, 3, CV_32F);
	cameraPoses[0].t = Mat::zeros(3, 1, CV_32F);

	// Go through spanning tree and for each camera calculate R and t
	// NOTE: This needs R, t of parent/previous camera
	if (tree.n <= 1) return;
	tree.walk([&cameraPoses, &images, &_log](const int i, const int j, const ImagePair* pair) -> bool
	{
		// jth camera is current leaf
		_log("Checking (%d > %d) to calculate R_%d", i, j, j);

		Mat R_ji, t_ji, R_j, t_j;
		Mat R_i = cameraPoses[i].R;
		Mat t_i = cameraPoses[i].t;

		if (pair->pair_index.first == i) {
			R_ji = pair->R;
			t_ji = pair->t;
		} else {
			Mat R_ij = pair->R;
			Mat t_ij = pair->t;
			R_ji = R_ij.t();
			t_ji = -t_ij;
		}

		// Calculate j-th (global) camera pose
		R_j = R_ji * R_i;
		t_j = t_ji + t_i;

		// Store calculated pose
		cameraPoses[j].R = R_j;
		cameraPoses[j].t = t_j;

		//std::cout << std::endl << j << std::endl <<
		//	R_j << std::endl << t_j << std::endl;

		//showImage("0", images[0].rgb);
		//showImageAndWait("j", images[j].rgb);

		// Continue
		return true;
	});

	_log.tok();
}

