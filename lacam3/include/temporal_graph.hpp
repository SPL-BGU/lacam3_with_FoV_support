/**
 * @file temporal_graph.hpp
 * @author Rotem Lev Lehman (levlerot@post.bgu.ac.il)
 * @brief A graph that contains vertices with timestamps where they are
 * available, representing a temporal graph.
 * @details This graph is used in the k-privacy post-processing step to
 * represent the safe zones of agents over time. Each vertex has a timestamp
 * indicating when it is available, allowing the algorithm to find paths that
 * keeps the agent within the safe zones at all times.
 * @version 0.1
 * @date 2025-06-24
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include "graph.hpp"
#include "sipp.hpp"

struct TemporalVertex {
    Vertex *vertex;            // Pointer to the original vertex
    std::set<int> timestamps;  // Timestamps when the vertex is available
    TemporalVertex(Vertex *v) : vertex(v), timestamps() {}
};

using TemporalVertices = std::vector<TemporalVertex *>;

struct TemporalGraph {
    TemporalVertices V;      // Temporal vertices
    Graph *G;                // Original graph
    int max_timestamp = -1;  // Maximum timestamp in the graph

    TemporalGraph(Graph *_G);
    TemporalGraph(const TemporalGraph &other);

    ~TemporalGraph();

    /**
     * @brief Add a timestep to a vertex in the temporal graph.
     *
     * @param v The vertex to which the timestep will be added.
     * @param timestamp The timestamp to be added to the vertex.
     */
    void add_timestep_to_vertex(Vertex *v, int timestamp);

    /**
     * @brief Convert the temporal graph to a safe interval table.
     *
     * @note This function is used for running the SIPP algorithm on the
     * TemporalGraph.
     *
     * @return SITable A safe interval table representing the safe intervals
     * for each vertex in the temporal graph.
     */
    SITable to_safe_interval_table() const;

    /**
     * @brief Calculates the distance from a given vertex to the closest safe
     * zone in the given timestamp.
     *
     * @param v The vertex to calculate the distance from.
     * @param timestamp The timestamp at which to calculate the distance.
     * @return double The distance from the vertex to the closest safe zone at
     * the given timestamp.
     */
    double distance_from_safe_zone(const Vertex *v, int timestamp) const;
};

class TemporalGraphConflictChecker : public ConflictChecker
{
  private:
    TemporalGraph *temporal_graph;

  public:
    TemporalGraphConflictChecker(TemporalGraph *_temporal_graph);
    /**
     * @brief The function checks if there is a conflict for the given vertex
     * at the given time.
     *
     * @details A conflict occurs if the vertex is not available at the
     * specified time, meaning that the agent cannot move to that vertex at that
     * time. If the v_from vertex is not available at the t_from time, it is an
     * error since the agent is trying to move from a vertex that is not
     * available - In that case, the function throws an exception.
     *
     * @param i The index of the agent - ignored in this implementation.
     * @param v_from The vertex from which the agent is moving.
     * @param v_to The vertex to which the agent is moving.
     * @param t_from The time at which the agent is moving from v_from to v_to.
     * @return true If there is a conflict.
     * @return false If there is no conflict.
     */
    bool is_conflict(const int i, const Vertex *v_from, const Vertex *v_to,
                     const int t_from) const override;
};

/**
 * @brief Prints the temporal graph into the given ostream.
 *
 * @param os The ostream to print into.
 * @param temporal_graph The temporal graph to print.
 * @return std::ostream& The ostream after the print.
 *
 * @details The print will look as follows:
 * ```
 * Temporal graph:
 * {}{}{}{}
 * {[1,4]}{}{}{}
 * {[1,3]}{[2,2],[4,7]}{}{}
 * ...
 * ```
 * Where each vertex has it's own brackets ({}) according to it's location, and
 * it contains all of the safe timesteps ranges in the brackets.
 */
std::ostream &operator<<(std::ostream &os, const TemporalGraph *temporal_graph);
