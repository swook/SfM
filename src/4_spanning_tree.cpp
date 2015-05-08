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

	// Build rest of tree
	const int old_n = assocMat.n;
	int new_n = 1;
	assocMat.walk([&tree, &new_n](const int j, ImagePair* pair) -> bool
	{
		// Find index of other side of pair
		const int i = pair->pair_index.first == j ?
		              pair->pair_index.second :
			      pair->pair_index.first;

		tree(i, j) = pair;
		tree(j, i) = pair;
		new_n++;

		return true; // continue
	});
	tree.n = new_n;

	// NOTE: Number of nodes (cameras) may be reduced if no edge from/to
	// camera exists
	_log("%d cameras reduced to %d cameras by spanning tree.", old_n, new_n);

	_log.tok();
}

