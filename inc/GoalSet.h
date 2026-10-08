#pragma once
#include <memory>
#include <vector>

// A stage goal that is a SET of vertices ("any cell labelled p0") instead of one vertex. Immutable and shared between CBS nodes.
// A single-vertex goal is the special case of a one-element set; CBSNode::goal_sets is empty for the legacy single-vertex search.
struct GoalSet
{
	std::vector<int> cells;   // the member vertices
	std::vector<char> member; // member[v] == 1 iff v is in the set (size = map size), O(1) membership for the search

	GoalSet(std::vector<int> c, int map_size) : cells(std::move(c)), member(map_size, 0)
	{
		for (int v : cells)
			if (v >= 0 && v < map_size) member[v] = 1;
	}
	bool contains(int v) const { return v >= 0 && v < (int) member.size() && member[v]; }
	bool empty() const { return cells.empty(); }
};
using GoalSetPtr = std::shared_ptr<const GoalSet>;
