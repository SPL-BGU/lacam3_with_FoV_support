#include "../include/temporal_graph.hpp"

#include <limits>

void TemporalGraph::add_timestep_to_vertex(Vertex *v, int timestamp)
{
    if (v == nullptr) {
        throw std::invalid_argument("The vertex cannot be nullptr.");
    }
    if (v->id < 0 || v->id >= V.size()) {
        throw std::out_of_range("Vertex ID is out of range in TemporalGraph.");
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
    V[v->id]->timestamps.insert(timestamp);
    timestemp_to_vertices_map[timestamp].push_back(v);

    if (timestamp > max_timestamp) {
        max_timestamp = timestamp;
    }
}

TemporalGraph::TemporalGraph(const Instance *_ins)
    : ins(_ins), G(_ins->G), V(_ins->G->V.size(), nullptr)
{
    for (auto v : G->V) {
        if (v != nullptr) {
            V[v->id] = new TemporalVertex(v);
        } else {
            throw std::runtime_error(
                "Graph's V contains nullptr, which is not allowed.");
        }
    }
}

TemporalGraph::TemporalGraph(const TemporalGraph &other)
    : ins(other.ins), G(other.G), max_timestamp(other.max_timestamp)
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
        if (v->timestamps.empty()) {
            // If the vertex has no timestamps, it is never safe.
            // Mark it with INT_MAX to indicate it is always unsafe.
            safe_interval_table.body[v->vertex->id] = safe_intervals;
            continue;
        }
        auto it = v->timestamps.begin();
        int start_time = *it;
        int end_time = start_time;
        if (it !=
            v->timestamps
                .end()) {  // Handle the case where there is only one timestamp.
            ++it;
        }

        while (it != v->timestamps.end()) {
            if (*it == end_time + 1) {
                end_time = *it;  // Extend the interval
            } else {
                safe_intervals.emplace_back(start_time, end_time);
                start_time = *it;
                end_time = start_time;
            }
            ++it;
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
        throw std::out_of_range("Timestamp is out of range in TemporalGraph.");
    }

    // Check if the vertex is in the field of view of any safe zone at the given
    // timestamp
    auto it = timestemp_to_vertices_map.find(timestamp);
    if (it == timestemp_to_vertices_map.end()) {
        return false;  // No vertices are safe at this timestamp
    }
    Vertices safe_vertices_at_timestamp = it->second;
    for (const auto &vertex : safe_vertices_at_timestamp) {
        if (in_field_of_view(vertex, v, ins->field_of_view_radius)) {
            return true;  // If the vertex is in the field of view of any
                          // vertex in the safe zone at the given timestamp,
                          // return true
        }
    }
    return false;
}

double TemporalGraph::distance_from_safe_zone(const Vertex *v,
                                              int timestamp) const
{
    if (v == nullptr) {
        throw std::invalid_argument("The vertex cannot be nullptr.");
    }
    if (v->id < 0 || v->id >= V.size()) {
        throw std::out_of_range("Vertex ID is out of range in TemporalGraph.");
    }
    if (timestamp < 0 || timestamp > max_timestamp) {
        throw std::out_of_range("Timestamp is out of range in TemporalGraph.");
    }

    // Find the closest safe zone at the given timestamp
    if (V[v->id]->timestamps.count(timestamp) > 0) {
        // If the vertex is safe at this timestamp, distance is 0
        return 0;
    }
    double min_distance = std::numeric_limits<double>::max();
    for (const auto &tv : V) {
        if (tv->timestamps.count(timestamp) > 0) {
            double distance = std::sqrt(std::pow(tv->vertex->x - v->x, 2) +
                                        std::pow(tv->vertex->y - v->y, 2));
            if (distance < min_distance) {
                min_distance = distance;
            }
        }
    }
    return min_distance;
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
    if (temporal_graph->V[v_from->id]->timestamps.count(t_from) == 0) {
        throw std::runtime_error(
            "The vertex v_from is not available at the specified time t_from.");
    }
    if (temporal_graph->V[v_to->id]->timestamps.count(t_from + 1) == 0) {
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
