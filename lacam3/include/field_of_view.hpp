/**
 * @file field_of_view.hpp
 * @author Rotem Lev Lehman (levlerot@post.bgu.ac.il)
 * @brief Field of view support for MAPF solvers.
 * @version 0.1
 * @date 2025-03-01
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include <set>
#include <stdexcept>
#include <vector>

#include "graph.hpp"

/**
 * @brief This class allows to count the number of agent groups that occupy a
 * node. Also, for each agent group, it allows to count the number of agents
 * from that group that occupy a node.
 *
 * @note It is used for the field of view of agents.
 */
class CounterWithSize
{
  private:
    std::vector<size_t> agent_groups;
    std::set<size_t> agents_occupied;
    size_t _size;
    size_t agent_groups_count;

  public:
    CounterWithSize(size_t _agent_groups_count)
        : agent_groups(_agent_groups_count, 0),
          agents_occupied(),
          _size(0),
          agent_groups_count(_agent_groups_count)
    {
    }

    /**
     * @brief The size of the counter for all agent groups.
     *
     * @return size_t The size of the counter.
     */
    size_t size() const { return _size; }

    /**
     * @brief Get the agents occupied set.
     *
     * @return std::set<size_t> The agents that see the node (in their field of
     * view).
     */
    std::set<size_t> get_agents_occupied() const { return agents_occupied; }

    /**
     * @brief Get the number of agents in the given agent group that occupy the
     * node.
     *
     * @param i The agent group id.
     * @return size_t The number of agents in the given agent group that occupy
     * the node.
     */
    size_t get_agent_group_count(size_t i) const
    {
        if (i >= agent_groups_count) {
            throw std::runtime_error("Index out of bounds.");
        }
        return agent_groups[i];
    }

    /**
     * @brief Checks if the field of view is occupied by the given agent.
     *
     * @param agent_id The agent id.
     * @return true If occupied by the agent.
     * @return false Otherwise.
     */
    bool is_occupied_by_agent(size_t agent_id) const
    {
        return agents_occupied.count(agent_id) > 0;
    }

    /**
     * @brief Increase the number of agents in the given agent group that occupy
     * the node by one.
     *
     * @param i The agent group id.
     */
    void occupy(size_t agent_id, size_t agent_group_id)
    {
        if (agent_group_id >= agent_groups_count) {
            throw std::runtime_error("Index out of bounds.");
        }
        if (agents_occupied.count(agent_id) > 0) {
            throw std::runtime_error(
                "The agent is already occupying the node.");
        }
        ++_size;
        ++agent_groups[agent_group_id];
        agents_occupied.insert(agent_id);
    }

    /**
     * @brief Decrease the number of agents in the given agent group that occupy
     * the node by one.
     *
     * @param i The agent group id.
     */
    void deoccupy(size_t agent_id, size_t agent_group_id)
    {
        if (agent_group_id >= agent_groups_count) {
            throw std::runtime_error("Index out of bounds.");
        }
        if (agents_occupied.count(agent_id) == 0) {
            throw std::runtime_error("The agent is not occupying the node.");
        }
        if (agent_groups[agent_group_id] == 0) {
            throw std::runtime_error("Cannot decrease from index " +
                                     std::to_string(agent_group_id) +
                                     " because it is already 0.");
        }
        --_size;
        --agent_groups[agent_group_id];
        agents_occupied.erase(agent_id);
    }

    std::string to_string() const
    {
        std::string str = "groups: [";
        for (size_t i = 0; i < agent_groups.size(); ++i) {
            str += std::to_string(agent_groups[i]) + ", ";
        }
        str += "], agents: [";
        for (size_t agent_id : agents_occupied) {
            str += std::to_string(agent_id) + ", ";
        }
        str += "]";
        return str;
    }
};

using Fields = std::vector<CounterWithSize>;

/**
 * @brief Calculate the number of agent groups.
 *
 * @param N The amount of agents.
 * @param k The size of each agent group.
 * @return size_t The number of agent groups.
 */
inline size_t get_num_of_agent_groups(size_t N, size_t k)
{
    if (N % k != 0) {
        throw std::runtime_error(
            "The given amount of agents must be divisible by k.\n");
    }
    return N / k;
}

/**
 * @brief Get the field of view from the current node location.
 *
 * @param g - The graph in which the nodes are from.
 * @param node - The node to calculate the field of view from.
 * @param field_radius - The field of view radius.
 * @return Vertices - The field of view from the current node location.
 */
Vertices get_field_of_view(Graph *g, Vertex *node, int field_radius);

/**
 * @brief Checks if the two given nodes are within each others field of view.
 *
 * @param n1 - First node.
 * @param n2 - Second node.
 * @param field_radius - The field of view radius.
 * @return true - If both nodes are in each others field of view.
 * @return false - Else (not in each others field of view).
 */
bool in_field_of_view(const Vertex *n1, const Vertex *n2, int field_radius);

/**
 * @brief Get the agent group id.
 *
 * @note The group is determined by the agent_id divided by k.
 *
 * @param agent_id The mock agent id.
 * @param k The amount of mock agents used for each agent.
 * @return size_t The group id.
 */
inline size_t get_agent_group_id(int agent, size_t k) { return agent / k; }
