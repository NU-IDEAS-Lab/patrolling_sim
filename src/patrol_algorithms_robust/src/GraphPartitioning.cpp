#include "patrol_algorithms_robust/GraphPartitioning.hpp"

// Returns the cost to travel along the edge connecting `a` and `b`.
// Cost is given for direction a->b.
// Returns 0 if a and b are not linked by an edge.
unsigned int graphGetEdgeCost(vertex *a, vertex *b)
{
    // For some reason the neighbors are stored as a list of IDs, not pointers.
    // Loop through to find the correct one.
    for(unsigned int i = 0; i < a->num_neigh; i ++)
    {
        if(a->id_neigh[i] == b->id)
        {
            return a->cost[i];
        }
    }
    return 0;
}

