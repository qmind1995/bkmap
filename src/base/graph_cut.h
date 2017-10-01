//
// Created by tri on 24/09/2017.
//

#ifndef BKMAP_GRAPH_CUT_H
#define BKMAP_GRAPH_CUT_H

#include <unordered_map>
#include <vector>

namespace bkmap {

// Compute the min-cut of a undirected graph using the Stoer Wagner algorithm.
    void ComputeMinGraphCut(const std::vector<std::pair<int, int>>& edges,
                            const std::vector<int>& weights, int* cut_weight,
                            std::vector<char>* cut_labels);

// Compute the normalized min-cut of an undirected graph using Graclus.
// Partitions the graph into clusters and returns the cluster labels per vertex.
    std::unordered_map<int, int> ComputeNormalizedMinGraphCut(
            const std::vector<std::pair<int, int>>& edges,
            const std::vector<int>& weights, const int num_parts);

}

#endif //BKMAP_GRAPH_CUT_H
