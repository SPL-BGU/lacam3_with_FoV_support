/*
 * fast collision checking, used in SUO and refinner
 */
#pragma once

#include "field_of_view.hpp"
#include "graph.hpp"
#include "instance.hpp"
#include "max_key_map.hpp"
#include "utils.hpp"

struct CollisionTable {
    // vertex, time, agents
    std::vector<std::vector<std::vector<int>>> body;
    std::vector<std::vector<int>> body_last;
    // Using this format to avoid sparse matrix.
    // Vertex, hash_map[timestep -> agents_viewing]
    std::vector<MaxKeyMap<int, std::vector<int>>> body_field_of_view;
    // The goal setting for the agents.
    // Vertex, vector[<agent_viewing, timestep_of_goal>]
    std::vector<std::vector<std::tuple<int, int>>> body_last_field_of_view;

    int collision_cnt;
    int N;

    const Instance *ins;

    CollisionTable(const Instance *_ins);
    ~CollisionTable();

    int getCollisionCost(const int i, const Vertex *v_from, const Vertex *v_to,
                         const int t_from);
    void enrollPath(const int i, Path &path);
    void clearPath(const int i, Path &path);
    void shrink();
};
