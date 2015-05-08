#include <list>

#include "opencv2/core.hpp"
using namespace cv;

#include "Pipeline.hpp"
#include "Associativity.hpp"
#include "util.hpp"

int _build_spanning_tree(const ImagePairs& pairs, Associativity& assocMat);

void Pipeline::build_spanning_tree(const ImagePairs& pairs, Associativity& assocMat)
{
	Logger _log("Step 4 (spanning)");

	// Build rest of tree
	// NOTE: Number of nodes (cameras) may be reduced if no edge from/to
	// camera exists
	const int old_n = assocMat.n;
	const int new_n = _build_spanning_tree(pairs, assocMat);
	_log("%d cameras reduced to %d cameras by spanning tree.", old_n, new_n);

	_log.tok();
}

int _build_spanning_tree(const ImagePairs& pairs, Associativity& assocMat)
{
	// Vector to check if camera has been added to queue before
	std::vector<bool> checkedCamera(assocMat.n);

	// Add first camera
	Associativity tree;
	checkedCamera[0] = true;
	int n = 1;

	std::list<int> queue;
	queue.push_back(0);
	while (!queue.empty())
	{
		int i = queue.front();
		queue.pop_front();

		pImagePairs found_pairs = assocMat.getAssociatedPairs(i);
		ImagePair* pair;
		for (int p = 0; p < found_pairs.size(); p++)
		{
			pair = found_pairs[p];
			int j = pair->pair_index.first != i ?
				pair->pair_index.first :
				pair->pair_index.second;

			if (i == j || checkedCamera[j]) continue; // Already got R,t

			//std::cout << i << " <-> " << j << std::endl;
			tree(i, j) = pair;
			tree(j, i) = pair;
			n++;
			checkedCamera[j] = true; // Done
			queue.push_back(j);
		}
	}

	tree.n = n;
	std::swap(assocMat, tree);
	return n;
}

