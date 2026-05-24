/*
 * implementation of PIBT
 *
 * references:
 * Priority Inheritance with Backtracking for Iterative Multi-agent Path
 * Finding. Keisuke Okumura, Manao Machida, Xavier Défago & Yasumasa Tamura.
 * Artificial Intelligence (AIJ). 2022.
 */
#pragma once
#include "dist_table.hpp"
#include "field_of_view.hpp"
#include "graph.hpp"
#include "instance.hpp"
#include "scatter.hpp"
#include "utils.hpp"

struct PIBT {
    const Instance *ins;
    std::mt19937 MT;

    // solver utils
    const int N;  // number of agents
    const int V_size;
    DistTable *D;

    // specific to PIBT
    const int NO_AGENT;
    std::vector<int> occupied_now;   // for quick collision checking
    std::vector<int> occupied_next;  // for quick collision checking
    std::vector<std::array<Vertex *, 5>> C_next;  // next location candidates
    std::vector<float> tie_breakers;              // random values, used in PIBT

    // field of view
    /**
     * @brief The field of view of agents in their current configuration.
     *
     * @note Format is <node-id, CounterWithSize>, whether the node is in
     *       the field of view of an agent or not.
     *
     * @note A node can be in the field of view of several agent groups.
     *
     * @note Works as a reservation table for quick collision checking.
     *
     */
    Fields occupied_field_of_view_now;
    /**
     * @brief The field of view of agents in their next configuration.
     *
     * @note Format is <node-id, CounterWithSize>, whether the node is in
     *       the field of view of an agent or not.
     *
     * @note A node can be in the field of view of several agent groups.
     *
     * @note Works as a reservation table for quick collision checking.
     *
     */
    Fields occupied_field_of_view_next;

    // swap, used in the LaCAM* paper
    bool flg_swap;

    // scatter
    Scatter *scatter;

    PIBT(const Instance *_ins, DistTable *_D, int seed = 0,
         bool _flg_swap = true, Scatter *_scatter = nullptr);
    ~PIBT();

    bool set_new_config(const Config &Q_from, Config &Q_to,
                        const std::vector<int> &order);
    bool _set_new_config(const Config &Q_from, Config &Q_to,
                         const std::vector<int> &order);
    /**
     * @brief A helper function for funcPIBT.
     * It tries to set the next vertex for i to be u by moving other conflicting
     * agents.
     *
     * @param i The agent to set the next vertex for.
     * @param Q_from The current configuration.
     * @param Q_to The next configuration.
     * @param u The vertex to set as the next vertex for i.
     * @return std::set<size_t> The set of agents that were moved to make room
     * for i (including i).
     */
    std::set<size_t> _set_next_vertex(const int i, const Config &Q_from,
                                      Config &Q_to, Vertex *u);
    std::set<size_t> funcPIBT(const int i, const Config &Q_from, Config &Q_to);
    int is_swap_required_and_possible(const int ai, const Config &Q_from,
                                      Config &Q_to);
    bool is_swap_required(const int pusher, const int puller,
                          Vertex *v_pusher_origin, Vertex *v_puller_origin);
    bool is_swap_possible(Vertex *v_pusher_origin, Vertex *v_puller_origin);

    /**
     * @brief Occupies the entire field of view of the given node to be of the
     * given agent.
     *
     * @note In order to de-occupy, call the deoccupy function.
     *
     * @param occupied The occupied nodes vector to occupy at.
     * @param occupied_field_of_view The occupied field of view vector to occupy
     * at.
     * @param node The node to occupy it's field of view.
     * @param agent The agent to occupy inside the field of view.
     *
     */
    void occupy(std::vector<int> &occupied, Fields &occupied_field_of_view,
                Vertex *node, int agent);

    /**
     * @brief De-occupies the entire field of view of the given node to be of
     * the given agent.
     *
     * @param occupied The occupied nodes vector to de-occupy at.
     * @param occupied_field_of_view The occupied field of view vector to
     * de-occupy at.
     * @param node The node to de-occupy it's field of view.
     * @param agent The agent to de-occupy inside the field of view.
     *
     */
    void deoccupy(std::vector<int> &occupied, Fields &occupied_field_of_view,
                  Vertex *node, int agent);

    /**
     * @brief Checks if the given agent can occupy the given node.
     *
     * @note A node can be occupied if it is not in the field of view of some
     * other agent's group, and not already occupied.
     *
     * @param occupied The occupied nodes vector to check if the node is
     * occupied.
     * @param occupied_field_of_view The occupied field of view vector to check
     * if the node is in the field of view of another agent's group.
     * @param node The node to check if it can be occupied.
     * @param agent The agent to check if can occupy the node.
     * @return true If the agent can occupy the node.
     * @return false Otherwise.
     */
    bool can_occupy(std::vector<int> &occupied, Fields &occupied_field_of_view,
                    Vertex *node, int agent);
};
