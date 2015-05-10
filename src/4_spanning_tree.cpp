#include <list>

#include "opencv2/core.hpp"
using namespace cv;

#include "Pipeline.hpp"
#include "Associativity.hpp"
#include "util.hpp"

void Pipeline::build_spanning_tree(const ImagePairs& pairs, Associativity& assocMat,
	Associativity& tree)
{
	Logger _log("Step 4 (spanning)");

	const int old_n = assocMat.n;

	// Only add camera once
	std::vector<bool> checked(old_n);
	checked[0] = true;

	// Build rest of tree
	int new_n = 1;
	assocMat.walk([&tree, &new_n, &checked](const int i, const int j, ImagePair* pair) -> bool
	{
		if (checked[j]) return true;

		tree(i, j) = pair;
		checked[j] = true;
		new_n++;

		return true; // continue
	});
	tree.n = new_n;

	// NOTE: Number of nodes (cameras) may be reduced if no edge from/to
	// camera exists
	_log("%d cameras reduced to %d cameras by spanning tree.", old_n, new_n);

	_log.tok();
}

