#include "Pipeline.hpp"
#include "Associativity.hpp"
#include "util.hpp"

void Pipeline::glo_cam_poses(CameraPoses& cameraPoses, const ImagePairs& pairs,
	Associativity& tree)
{
	Logger _log("Step 5 (global)");
	_log("Start");

	tree.walk([&cameraPoses](const ImagePair* pair) -> bool
	{
		return true;
	});

	_log.tok();
}

