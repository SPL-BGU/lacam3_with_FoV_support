#include "../include/collision_table.hpp"

CollisionTable::CollisionTable(const Instance *_ins, const Deadline *_deadline)
    : ins(_ins),
      deadline(_deadline),
      body(_ins->G->size()),
      body_last(_ins->G->size()),
      collision_cnt(0),
      N(_ins->N),
      body_field_of_view(_ins->G->size()),
      body_last_field_of_view(_ins->G->size())
{
}

CollisionTable::~CollisionTable() {}

int other_groups_count(const std::unordered_set<int> &agents, const int i,
                       const Instance *ins)
{
    size_t group_id = get_agent_group_id(i, ins->k);
    size_t count = 0;
    for (auto agent : agents) {
        if (agent != i && get_agent_group_id(agent, ins->k) != group_id) {
            ++count;
        }
    }
    return count;
}

int CollisionTable::getCollisionCost(const int i, const Vertex *v_from,
                                     const Vertex *v_to, const int t_from)
{
    const int t_to = t_from + 1;
    auto collision = 0;

    // vertex collision
    if (t_to < body[v_to->id].size()) {
        collision += body[v_to->id][t_to].size();
    }

    // edge collision
    if (t_to < body[v_from->id].size() && t_from < body[v_to->id].size()) {
        for (auto j : body[v_from->id][t_to]) {
            for (auto k : body[v_to->id][t_from]) {
                if (j == k) ++collision;
            }
        }
    }

    // field of view collision
    if (body_field_of_view[v_to->id].contains(t_to)) {
        collision += other_groups_count(
            body_field_of_view[v_to->id].getMap()[t_to], i, ins);
    }

    // goal collision
    for (auto last_timestep : body_last[v_to->id]) {
        if (t_to > last_timestep) ++collision;
    }

    // field of view goal collision
    size_t group_id = get_agent_group_id(i, ins->k);
    for (auto &[agent, last_timestep] : body_last_field_of_view[v_to->id]) {
        if (t_to > last_timestep) {
            if (agent != i && get_agent_group_id(agent, ins->k) != group_id) {
                ++collision;
            }
        }
    }

    return collision;
}

void occupy_vertex_field_of_view(
    const Instance *ins, Vertex *v, int agent, int timestep,
    std::vector<MaxKeyMap<int, std::unordered_set<int>>> &body_field_of_view)
{
    Vertices field_of_view_vertices =
        get_field_of_view(ins->G, v, ins->field_of_view_radius);

    for (const Vertex *current_vertex : field_of_view_vertices) {
        // Register field of view
        if (!body_field_of_view[current_vertex->id].contains(timestep)) {
            // If the timestep does not exist, create a new entry with an empty
            // vector.
            body_field_of_view[current_vertex->id].insert(
                timestep, std::unordered_set<int>());
        }
        body_field_of_view[current_vertex->id].getMap()[timestep].insert(agent);
    }
}

void deoccupy_vertex_field_of_view(
    const Instance *ins, Vertex *v, int agent, int timestep,
    std::vector<MaxKeyMap<int, std::unordered_set<int>>> &body_field_of_view)
{
    Vertices field_of_view_vertices =
        get_field_of_view(ins->G, v, ins->field_of_view_radius);

    for (const Vertex *current_vertex : field_of_view_vertices) {
        std::unordered_set<int> &agents_viewing =
            body_field_of_view[current_vertex->id].getMap()[timestep];
        auto it = agents_viewing.find(agent);

        if (it != agents_viewing.end()) {
            // Remove the agent from the field of view
            agents_viewing.erase(it);
        } else {
            throw std::runtime_error(
                "The vertex is not occupied by the given agent.\n");
        }
    }
}

bool CollisionTable::enrollPath(const int i, Path &path)
{
    if (path.empty()) return true;
    const auto T_i = path.size() - 1;

    for (auto t = 0; t <= T_i; ++t) {
        if (is_expired(deadline)) return false;  // time limit exceeded
        auto v = path[t];

        // update collision count
        if (t > 0)
            collision_cnt += getCollisionCost(i, path[t - 1], path[t], t - 1);

        // register
        while (body[v->id].size() <= t) body[v->id].emplace_back();
        body[v->id][t].push_back(i);

        // register field of view
        occupy_vertex_field_of_view(ins, v, i, t, body_field_of_view);
    }

    // goal
    body_last[path.back()->id].push_back(T_i);
    auto &&entry = body[path.back()->id];
    for (auto t = T_i + 1; t < entry.size(); ++t) {
        collision_cnt += entry[t].size();
    }

    // register field of view for the goal
    Vertices field_of_view_vertices =
        get_field_of_view(ins->G, path.back(), ins->field_of_view_radius);

    for (const Vertex *current_vertex : field_of_view_vertices) {
        if (is_expired(deadline)) return false;  // time limit exceeded
        // Register field of view
        body_last_field_of_view[current_vertex->id].emplace_back(i, T_i);
        // Count collisions for future timesteps (with goal vertex).
        auto &&entry_field_of_view = body_field_of_view[current_vertex->id];
        for (auto t = T_i + 1; t <= entry_field_of_view.getMaxKey().value_or(0);
             ++t) {
            if (is_expired(deadline)) return false;  // time limit exceeded
            if (entry_field_of_view.contains(t)) {
                collision_cnt +=
                    other_groups_count(entry_field_of_view.getMap()[t], i, ins);
            }
        }
    }

    return true;
}

bool CollisionTable::clearPath(const int i, Path &path)
{
    if (path.empty()) return true;
    const auto T_i = (int)path.size() - 1;

    for (auto t = 0; t <= T_i; ++t) {
        if (is_expired(deadline)) return false;  // time limit exceeded
        auto v = path[t];
        auto &&entry = body[v->id][t];

        // remove field of view.
        deoccupy_vertex_field_of_view(ins, v, i, t, body_field_of_view);

        // remove entry
        for (auto itr = entry.begin(); itr != entry.end();) {
            if (*itr == i) {
                entry.erase(itr);
                break;
            } else {
                ++itr;
            }
        }

        // update collision count
        if (t > 0)
            collision_cnt -= getCollisionCost(i, path[t - 1], path[t], t - 1);
    }

    // goal
    auto &&entry_body_last = body_last[path.back()->id];
    for (auto itr = entry_body_last.begin(); itr != entry_body_last.end();) {
        if (*itr == T_i) {
            entry_body_last.erase(itr);
            break;
        } else {
            ++itr;
        }
    }
    auto &&entry_body = body[path.back()->id];
    for (auto t = T_i + 1; t < entry_body.size(); ++t) {
        collision_cnt -= entry_body[t].size();
    }

    // remove field of view for the goal
    Vertices field_of_view_vertices =
        get_field_of_view(ins->G, path.back(), ins->field_of_view_radius);
    for (const Vertex *current_vertex : field_of_view_vertices) {
        if (is_expired(deadline)) return false;  // time limit exceeded
        // Remove field of view
        body_last_field_of_view[current_vertex->id].erase(
            std::remove_if(body_last_field_of_view[current_vertex->id].begin(),
                           body_last_field_of_view[current_vertex->id].end(),
                           [i](const std::tuple<int, int> &entry) {
                               return std::get<0>(entry) == i;
                           }),
            body_last_field_of_view[current_vertex->id].end());
        // Remove collisions for future timesteps (with goal vertex).
        auto &&entry_field_of_view = body_field_of_view[current_vertex->id];
        for (auto t = T_i + 1; t <= entry_field_of_view.getMaxKey().value_or(0);
             ++t) {
            if (is_expired(deadline)) return false;  // time limit exceeded
            if (entry_field_of_view.contains(t)) {
                collision_cnt -=
                    other_groups_count(entry_field_of_view.getMap()[t], i, ins);
            }
        }
    }
    return true;
}
