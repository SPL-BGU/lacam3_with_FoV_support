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

#include <boost/dynamic_bitset.hpp>

#include "graph.hpp"
#include "sipp.hpp"

struct TemporalVertex {
    Vertex *vertex;  // Pointer to the original vertex
    boost::dynamic_bitset<>
        timestamps;  // Timestamps when the vertex is available
    TemporalVertex(Vertex *v, int max_timestamp)
        : vertex(v), timestamps(max_timestamp + 1){};
};

using TemporalVertices = std::vector<TemporalVertex *>;

struct TemporalGraph {
    TemporalVertices V;  // Temporal vertices
    Graph *G;            // Original graph
    int max_timestamp;   // Maximum timestamp in the graph
    const Instance *ins;
    // For quick lookup of vertices available at a specific timestamp holds the
    // vertices indexes as a bitset for efficient storage and fast operations.
    std::vector<boost::dynamic_bitset<>> timestamp_to_vertices_map;

    TemporalGraph(const Instance *_ins, int _max_timestamp);
    TemporalGraph(const TemporalGraph &other);

    ~TemporalGraph();

    /**
     * @brief Add a timestep to a vertex in the temporal graph.
     *
     * @note add_to_temporal_vertex_timestamps is false
     * only when we are extending the safe zones,
     * since then we will use multiple threads and we don't want to have
     * a race condition on the timestamps bitset of the TemporalVertex.
     * On that case, we will update this bitset in the end of the extension
     * process in a single thread using the function
     * complete_temporal_vertex_timestamps.
     *
     * @param v The vertex to which the timestep will be added.
     * @param timestamp The timestamp to be added to the vertex.
     * @param add_to_temporal_vertex_timestamps Whether to add the timestamp to
     * the TemporalVertex's timestamps bitset.
     */
    void add_timestep_to_vertex(Vertex *v, int timestamp,
                                bool add_to_temporal_vertex_timestamps = true);

    /**
     * @brief Completes the timestamps bitset of each TemporalVertex
     * in the temporal graph based on the timestamp_to_vertices_map.
     *
     */
    void complete_temporal_vertex_timestamps();

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
     * @brief Checks if v is in the field of view of any vertex in the safe zone
     * at the given timestamp.
     *
     * @param v The vertex to check.
     * @param timestamp The timestamp at which to check the safe zone.
     * @return true If v is in the field of view of any vertex in the safe zone
     * at the given timestamp.
     * @return false Otherwise.
     */
    bool is_in_field_of_view_of_safe_zone(const Vertex *v, int timestamp) const;
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
