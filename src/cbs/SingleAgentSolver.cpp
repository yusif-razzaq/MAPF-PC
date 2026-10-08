#include "SingleAgentSolver.h"
#include <queue>
#include <functional>


list<int> SingleAgentSolver::getNextLocations(int curr) const // including itself and its neighbors
{
	list<int> rst = instance.getNeighbors(curr);
	rst.emplace_back(curr);
	return rst;
}

// JK: This is a TO-DO
void SingleAgentSolver::compute_heuristics(const vector<int>& goals)
{
  set_mode_ = false;
	struct Node
	{
		int location;
		int value;

		Node() = default;
		Node(int location, int value) : location(location), value(value) {}

		// the following is used to compare nodes in the OPEN list
		struct compare_node
		{
			// returns true if n1 > n2 (note -- this gives us *min*-heap).
			bool operator()(const Node& n1, const Node& n2) const
			{
				return n1.value >= n2.value;
			}
		};  // used by OPEN (heap) to compare nodes (top of the heap has min f-val, and then highest g-val)
	};

  my_heuristic.resize(goals.size());
  heuristic_landmark.resize(goals.size(), 0);

  for (int i = 0; i < goals.size(); i++){
    my_heuristic[i].resize(instance.map_size, MAX_TIMESTEP);

    // generate a heap that can save nodes (and a open_handle)
    boost::heap::pairing_heap<Node, boost::heap::compare<Node::compare_node>> heap;
    Node root(goals[i], 0);
    my_heuristic[i][goals[i]] = 0;
    heap.push(root);  // add root to heap
    while (!heap.empty())
      {
        Node curr = heap.top();
        heap.pop();
        for (int next_location : instance.getNeighbors(curr.location))
          {
            if (my_heuristic[i][next_location] > curr.value + 1)
              {
                my_heuristic[i][next_location] = curr.value + 1;
                Node next(next_location, curr.value + 1);
                heap.push(next);
              }
          }
      }
  }


  for (int i = goals.size() - 2; i >= 0; i--){
    heuristic_landmark[i] = heuristic_landmark[i + 1] + my_heuristic[i + 1][goals[i]];
  }

}


void SingleAgentSolver::compute_heuristics(const vector<GoalSetPtr>& sets)
{
  set_mode_ = true;
  const int S = (int) sets.size();
  const int N = (int) instance.map_size;
  my_heuristic.assign(S, vector<int>(N, MAX_TIMESTEP));
  heur_total_.assign(S, vector<int>(N, MAX_TIMESTEP));
  heuristic_landmark.assign(S, 0);

  using QE = std::pair<int, int>; // (value, location)
  auto dijkstra = [&](vector<int>& dist, const vector<QE>& seeds) {
    std::priority_queue<QE, vector<QE>, std::greater<QE>> pq;
    for (const auto& sd : seeds)
      if (sd.first < dist[sd.second]) { dist[sd.second] = sd.first; pq.push(sd); }
    while (!pq.empty())
    {
      auto [d, u] = pq.top();
      pq.pop();
      if (d > dist[u]) continue;
      for (int w : instance.getNeighbors(u))
        if (dist[w] > d + 1) { dist[w] = d + 1; pq.push({d + 1, w}); }
    }
  };

  for (int s = S - 1; s >= 0; s--)
  {
    // plain distance to the set
    vector<QE> seeds;
    for (int u : sets[s]->cells) seeds.push_back({0, u});
    dijkstra(my_heuristic[s], seeds);
    // exact remaining cost: seed every member with the cost of finishing the later stages from there
    seeds.clear();
    for (int u : sets[s]->cells)
      seeds.push_back({s == S - 1 ? 0 : heur_total_[s + 1][u], u});
    dijkstra(heur_total_[s], seeds);
    // cheapest remaining cost starting from any cell of this stage (what the single-vertex landmark sum is for a one-element set)
    int best = MAX_TIMESTEP;
    if (s < S - 1)
      for (int u : sets[s]->cells) best = std::min(best, heur_total_[s + 1][u]);
    heuristic_landmark[s] = (s == S - 1) ? 0 : best;
  }
}
