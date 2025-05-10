/*
 * fast collision checking, used in SUO and refinner
 */
#pragma once

#include "field_of_view.hpp"
#include "graph.hpp"
#include "instance.hpp"
#include "utils.hpp"

struct CollisionTable {
    // vertex, time, agents
    std::vector<std::vector<std::vector<int>>> body;
    std::vector<std::vector<int>> body_last;
    // time, field <vertex, counter_with_size>
    std::vector<Fields> body_field_of_view;

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
