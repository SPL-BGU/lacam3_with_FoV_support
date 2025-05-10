#include "../include/collision_table.hpp"

void initialize_field_of_view(Fields &field_of_view, const Instance *ins)
{
    size_t count = get_num_of_agent_groups(ins->N, ins->k);
    for (size_t i = 0; i < ins->G->size(); ++i) {
        field_of_view.push_back(CounterWithSize(count));
    }
}

CollisionTable::CollisionTable(const Instance *_ins)
    : ins(_ins),
      body(_ins->G->size()),
      body_last(_ins->G->size()),
      collision_cnt(0),
      N(_ins->N),
      body_field_of_view()
{
    // Initialize the first timestep.
    body_field_of_view.push_back(Fields());
    initialize_field_of_view(body_field_of_view[0], ins);
}

CollisionTable::~CollisionTable() {}

/**
 * @brief Get amount of agents from different groups in the field of view of the
 * vertex.
 *
 * @param field_of_view The field of view of all vertices.
 * @param i The agent id.
 * @param ins The instance.
 * @param v The vertex to check.
 * @return size_t The amount of agents from different groups in the field of
 * view of the vertex.
 */
size_t other_groups_count(const Fields &field_of_view, const int i,
                          const Instance *ins, const Vertex *v)
{
    size_t group_id = get_agent_group_id(i, ins->k);
    return field_of_view[v->id].size() -
           field_of_view[v->id].get_agent_group_count(group_id);
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
    if (t_to < body_field_of_view.size()) {
        collision += other_groups_count(body_field_of_view[t_to], i, ins, v_to);
    } else {
        // If the checked timestep is bigger than all other plans max timestep,
        // then all agents are considered at their goal and still need to be
        // checked as if they stay in goal.
        collision +=
            other_groups_count(body_field_of_view.back(), i, ins, v_to);
    }

    // goal collision
    for (auto last_timestep : body_last[v_to->id]) {
        if (t_to > last_timestep) ++collision;
    }
    return collision;
}

void occupy_vertex_field_of_view(const Instance *ins, Vertex *v, int agent,
                                 Fields &field_of_view)
{
    Vertices field_of_view_vertices =
        get_field_of_view(ins->G, v, ins->field_of_view_radius);
    size_t group_id = get_agent_group_id(agent, ins->k);

    for (const Vertex *current_vertex : field_of_view_vertices) {
        field_of_view[current_vertex->id].occupy(agent, group_id);
    }
}

void deoccupy_vertex_field_of_view(const Instance *ins, Vertex *v, int agent,
                                   Fields &field_of_view)
{
    Vertices field_of_view_vertices =
        get_field_of_view(ins->G, v, ins->field_of_view_radius);
    size_t group_id = get_agent_group_id(agent, ins->k);

    for (const Vertex *current_vertex : field_of_view_vertices) {
        if (field_of_view[current_vertex->id].is_occupied_by_agent(agent)) {
            field_of_view[current_vertex->id].deoccupy(agent, group_id);
        } else {
            throw std::runtime_error(
                "The vertex is not occupied by the given agent.\n");
        }
    }
}

void CollisionTable::enrollPath(const int i, Path &path)
{
    if (path.empty()) return;
    const auto T_i = path.size() - 1;

    // Add more timesteps if needed to the body_field_of_view.
    for (auto t = body_field_of_view.size(); t <= T_i; ++t) {
        // Copy the previous latest field of view, since all other
        // paths have already ended, and so their agents remain in place.
        body_field_of_view.emplace_back(body_field_of_view.back());
    }

    for (auto t = 0; t <= T_i; ++t) {
        auto v = path[t];

        // update collision count
        if (t > 0)
            collision_cnt += getCollisionCost(i, path[t - 1], path[t], t - 1);

        // register
        while (body[v->id].size() <= t) body[v->id].emplace_back();
        body[v->id][t].push_back(i);

        // register field of view
        occupy_vertex_field_of_view(ins, v, i, body_field_of_view[t]);
    }
    for (auto t = T_i + 1; t < body_field_of_view.size(); ++t) {
        // register field of view for the goal for the rest of the path.
        occupy_vertex_field_of_view(ins, path[T_i], i, body_field_of_view[t]);
    }

    // goal
    body_last[path.back()->id].push_back(T_i);
    auto &&entry = body[path.back()->id];
    for (auto t = T_i + 1; t < entry.size(); ++t) {
        collision_cnt += entry[t].size();
    }
}

void CollisionTable::clearPath(const int i, Path &path)
{
    if (path.empty()) return;
    const auto T_i = (int)path.size() - 1;
    for (auto t = T_i + 1; t < body_field_of_view.size(); ++t) {
        // remove field of view from goal of the removed path.
        deoccupy_vertex_field_of_view(ins, path[T_i], i, body_field_of_view[t]);
    }
    for (auto t = 0; t <= T_i; ++t) {
        auto v = path[t];
        auto &&entry = body[v->id][t];

        // remove field of view.
        deoccupy_vertex_field_of_view(ins, v, i, body_field_of_view[t]);

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
}
