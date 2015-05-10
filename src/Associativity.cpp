#include <list>

#include "Associativity.hpp"
#include "util.hpp"

Associativity::PairIndex Associativity::makeIndex(int a, int b) const
{
	int i = std::min(a, b),
	    j = i == a ? b : a;

	return PairIndex(i, j);
}

ImagePair*& Associativity::operator()(const int i, const int j)
{
	return _map[makeIndex(i, j)];
}

ImagePair* Associativity::operator()(const int i, const int j) const
{
	auto found = _map.find(makeIndex(i, j));
	if (found != _map.end()) return found->second;
	else                     return NULL;
}

pImagePairs Associativity::getAssociatedPairs(const int i)
{
	pImagePairs pairs;

	ImagePair* pair = NULL;
	for (int j = 0; j < n; j++)
	{
		if (i == j) continue;
		pair = (*this)(i, j);
		if (pair != NULL) pairs.push_back(pair);
	}
	return pairs;
}

void Associativity::walk(walk_func func)
{
	std::unordered_map<PairIndex, bool> checked;

	std::list<int> queue;
	queue.push_back(0);
	while (!queue.empty())
	{
		int i = queue.front();
		queue.pop_front();

		pImagePairs pairs = getAssociatedPairs(i);
		for (int p = 0; p < pairs.size(); p++)
		{
			ImagePair* pair = pairs[p];
			int j = pair->pair_index.first == i ?
				pair->pair_index.second :
				pair->pair_index.first;

			PairIndex chk_idx = makeIndex(i, j);
			if (checked[chk_idx]) continue;

			bool _continue = func(j, pair);
			checked[chk_idx] = true;
			if (_continue) queue.push_back(j);
		}
	}
}
