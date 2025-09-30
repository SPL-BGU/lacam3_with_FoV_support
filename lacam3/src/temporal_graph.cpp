#include "../include/temporal_graph.hpp"

#include <limits>

void TemporalGraph::add_timestep_to_vertex(
    Vertex *v, int timestamp, bool add_to_temporal_vertex_timestamps)
{
    if (v == nullptr) {
        throw std::invalid_argument("The vertex cannot be nullptr.");
    }
    if (v->id < 0 || v->id >= V.size()) {
        throw std::out_of_range("Vertex ID is out of range in TemporalGraph.");
    }
    if (timestamp < 0 || timestamp > max_timestamp) {
        throw std::out_of_range(
            "add_timestep_to_vertex: Timestamp (" + std::to_string(timestamp) +
            ") is out of range [0, " + std::to_string(max_timestamp) +
            "] in TemporalGraph.");
    }
    if (V[v->id] == nullptr) {
        throw std::runtime_error("The vertex (id = " + std::to_string(v->id) +
                                 ") is not initialized in the temporal graph.");
    }
    if (V[v->id]->vertex->id != v->id) {
        throw std::runtime_error(
            "The given vertex is not in the correct place in the temporal "
            "graph.");
    }
    if (add_to_temporal_vertex_timestamps) {
        V[v->id]->timestamps[timestamp] = 1;
    }
    timestamp_to_vertices_map[timestamp][v->id] = 1;
}

void TemporalGraph::complete_temporal_vertex_timestamps()
{
    for (size_t t = 0; t < timestamp_to_vertices_map.size(); ++t) {
        const auto &bitset = timestamp_to_vertices_map[t];
        for (size_t v_id = bitset.find_first();
             v_id != boost::dynamic_bitset<>::npos;
             v_id = bitset.find_next(v_id)) {
            if (V[v_id] == nullptr) {
                throw std::runtime_error(
                    "The vertex (id = " + std::to_string(v_id) +
                    ") is not initialized in the temporal graph.");
            }
            V[v_id]->timestamps[t] = 1;
        }
    }
}

TemporalGraph::TemporalGraph(const Instance *_ins, int _max_timestamp)
    : ins(_ins),
      G(_ins->G),
      V(_ins->G->V.size(), nullptr),
      max_timestamp(_max_timestamp),
      timestamp_to_vertices_map(_max_timestamp + 1,
                                boost::dynamic_bitset<>(_ins->G->V.size()))
{
    for (auto v : G->V) {
        if (v != nullptr) {
            V[v->id] = new TemporalVertex(v, max_timestamp);
        } else {
            throw std::runtime_error(
                "Graph's V contains nullptr, which is not allowed.");
        }
    }
}

TemporalGraph::TemporalGraph(const TemporalGraph &other)
    : ins(other.ins),
      G(other.G),
      max_timestamp(other.max_timestamp),
      timestamp_to_vertices_map(other.timestamp_to_vertices_map)
{
    V.resize(other.V.size(), nullptr);
    for (size_t i = 0; i < other.V.size(); ++i) {
        if (other.V[i] != nullptr) {
            V[i] = new TemporalVertex(*other.V[i]);
        } else {
            throw std::runtime_error(
                "Graph's V contains nullptr, which is not allowed.");
        }
    }
}

TemporalGraph::~TemporalGraph()
{
    for (auto &v : V)
        if (v != nullptr) delete v;
    V.clear();
}

SITable TemporalGraph::to_safe_interval_table() const
{
    SITable safe_interval_table(nullptr);

    for (const auto &v : V) {
        SIs safe_intervals;
        if (v->timestamps.none()) {
            // If the vertex has no timestamps, it is never safe.
            safe_interval_table.body[v->vertex->id] = safe_intervals;
            continue;
        }
        size_t start_time = v->timestamps.find_first();
        size_t end_time = start_time;
        for (auto t = v->timestamps.find_next(start_time);
             t != boost::dynamic_bitset<>::npos;
             t = v->timestamps.find_next(t)) {
            if (t == end_time + 1) {
                end_time = t;  // Extend the interval
            } else {
                safe_intervals.emplace_back(start_time, end_time);
                start_time = t;
                end_time = start_time;
            }
        }
        // Add the last interval.
        // If the end_time is the maximum timestamp,
        // we can extend it to INT_MAX - 1, as the SIPP algorithm expects this.
        if (end_time == max_timestamp) {
            end_time = INT_MAX - 1;
        }
        safe_intervals.emplace_back(start_time, end_time);
        safe_interval_table.body[v->vertex->id] = safe_intervals;
    }
    return safe_interval_table;
}

bool TemporalGraph::is_in_field_of_view_of_safe_zone(const Vertex *v,
                                                     int timestamp) const
{
    if (v == nullptr) {
        throw std::invalid_argument("The vertex cannot be nullptr.");
    }
    if (v->id < 0 || v->id >= V.size()) {
        throw std::out_of_range("Vertex ID is out of range in TemporalGraph.");
    }
    if (timestamp < 0 || timestamp > max_timestamp) {
        throw std::out_of_range(
            "is_in_field_of_view_of_safe_zone: Timestamp (" +
            std::to_string(timestamp) + ") is out of range [0, " +
            std::to_string(max_timestamp) + "] in TemporalGraph.");
    }

    // Check if the vertex is in the field of view of any safe zone at the given
    // timestamp
    auto safe_vertices_at_timestamp = timestamp_to_vertices_map[timestamp];
    auto fov = get_field_of_view_bitset(ins->G, v, ins->field_of_view_radius);
    return safe_vertices_at_timestamp.intersects(fov);
}

TemporalGraphConflictChecker::TemporalGraphConflictChecker(
    TemporalGraph *_temporal_graph)
    : temporal_graph(_temporal_graph)
{
}

bool TemporalGraphConflictChecker::is_conflict(const int i,
                                               const Vertex *v_from,
                                               const Vertex *v_to,
                                               const int t_from) const
{
    if (!temporal_graph->timestamp_to_vertices_map[t_from][v_from->id]) {
        throw std::runtime_error(
            "The vertex v_from is not available at the specified time t_from.");
    }
    if (!temporal_graph->timestamp_to_vertices_map[t_from + 1][v_to->id]) {
        return true;  // Conflict if either vertex is not available at the given
                      // time
    }
    return false;  // No conflict
}

std::ostream &operator<<(std::ostream &os, const TemporalGraph *temporal_graph)
{
    if (!temporal_graph) {
        os << "Temporal graph: nullptr" << std::endl;
        return os;
    }
    SITable safe_intervals_table = temporal_graph->to_safe_interval_table();

    os << "Temporal graph start" << std::endl;

    for (int y = 0; y < temporal_graph->G->height; y++) {
        for (int x = 0; x < temporal_graph->G->width; x++) {
            int index = temporal_graph->G->width * y + x;
            Vertex *v = temporal_graph->G->U[index];
            if (nullptr == v) {
                os << "{}";
            } else {
                SIs &safe_intervals = safe_intervals_table.get(v);
                os << "{";
                for (const auto &si : safe_intervals) {
                    os << "[" << si.first << "," << si.second << "]";
                    if (&si != &safe_intervals.back()) {
                        os << ",";
                    }
                }
                os << "}";
            }
        }
        os << std::endl;
    }
    os << "Temporal graph end" << std::endl;

    return os;
}
