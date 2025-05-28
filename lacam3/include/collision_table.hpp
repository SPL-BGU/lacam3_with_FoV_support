/*
 * fast collision checking, used in SUO and refinner
 */
#pragma once

#include <unordered_set>

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
    std::vector<MaxKeyMap<int, std::unordered_set<int>>> body_field_of_view;
    // The goal setting for the agents.
    // Vertex, vector[<agent_viewing, timestep_of_goal>]
    std::vector<std::vector<std::tuple<int, int>>> body_last_field_of_view;

    int collision_cnt;
    int N;

    const Instance *ins;
    const Deadline *deadline;

    CollisionTable(const Instance *_ins, const Deadline *_deadline = nullptr);
    ~CollisionTable();

    int getCollisionCost(const int i, const Vertex *v_from, const Vertex *v_to,
                         const int t_from);
    /**
     * @brief Enrolls a path for the agent i.
     *
     * @param i The index of the agent.
     * @param path The path to enroll.
     * @return true Everything is fine.
     * @return false Time limit exceeded.
     */
    bool enrollPath(const int i, Path &path);
    /**
     * @brief Clears the path for the agent i.
     *
     * @param i The index of the agent.
     * @param path The path to clear.
     * @return true Everything is fine.
     * @return false Time limit exceeded.
     */
    bool clearPath(const int i, Path &path);
    void shrink();
};
