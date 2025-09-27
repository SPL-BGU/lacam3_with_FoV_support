/**
 * @file k_privacy_post_process.hpp
 * @author Rotem Lev Lehman (levlerot@post.bgu.ac.il)
 * @brief This file contains the definition of the KPrivacyPostProcess class,
 * which is responsible for post-processing the solution of a multi-agent path
 * finding problem to ensure k-privacy.
 * @details It computes safe zones for agents and refines the path of a real
 * agent based on these safe zones.
 * @version 0.1
 * @date 2025-06-24
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include "dist_table.hpp"
#include "graph.hpp"
#include "instance.hpp"
#include "temporal_graph.hpp"
#include "utils.hpp"

class KPrivacyPostProcess
{
  private:
    const Instance *ins;
    DistTable *D;
    const int verbose;
    std::mt19937 MT;
    const Deadline *deadline;
    /**
     * @brief A map that caches the safe zones for each agent group id.
     *
     */
    std::unordered_map<int, TemporalGraph *> initial_safe_zones_cache;

    /**
     * @brief A vector that holds the enhanced safe zones for each agent group.
     *
     * @details This vector is used to store the safe zones after they have been
     * enhanced to include additional vertices that are not safe for any agent
     * group, but are close to the safe zones of the closest agent group.
     *
     */
    std::vector<TemporalGraph *> extended_safe_zones_list;

    /**
     * @brief Get the cached safe zones for the given agent group id.
     *
     * @note If the safe zones are not cached, it returns nullptr.
     *
     * @param agent_group_id The group id of the agent.
     * @return TemporalGraph* The cached safe zones for the agent group id, or
     * nullptr if not cached.
     */
    TemporalGraph *_get_initial_safe_zones(int agent_group_id);

  public:
    KPrivacyPostProcess(const Instance *instance, DistTable *_D, int seed = 0,
                        int verbosity = 0, const Deadline *_deadline = nullptr);
    ~KPrivacyPostProcess();

    /**
     * @brief Runs a single agent planner to find the shortest path that
     * goes through the safe zones (temporal graph) for the real agent.
     *
     * @param safe_zones the safe zones to find shortest path on.
     * @param agent_group_id the group id of the agent.
     * @param real_agent_id the id of the real agent in the group (to refine
     * it's path).
     * @return Path the refined path for the real agent.
     */
    Path refine_real_agent_path(TemporalGraph *safe_zones, int agent_group_id,
                                int real_agent_id);

    /**
     * @brief Computes safe zones for the agent's group based on the given
     * solution, and returns a temporal graph defining the safe zones.
     *
     * @param solution the original solution.
     * @param agent_group_id the group id of the agent.
     * @return TemporalGraph* a temporal graph defining the safe zones for the
     * agent's group.
     */
    TemporalGraph *get_initial_safe_zones(const Solution &solution,
                                          int agent_group_id);

    /**
     * @brief Returns the extended safe zones of the given agent group.
     *
     * @param agent_group_id the group id of the agent.
     * @return TemporalGraph* a temporal graph defining the safe zones for the
     * agent's group.
     * @throw std::runtime_error If it is not initialized using the
     * initialize_extended_safe_zones_cache function.
     */
    TemporalGraph *get_extended_safe_zones(int agent_group_id);

    /**
     * @brief Computes and caches the safe zones for all agent groups,
     * and extend them to closest locations which are not safe for any agent
     * group. This is done to enable planning more flexible paths for the agents
     * using close but unused vertices.
     *
     * @details See _extend_safe_zone function for the rules of extending the
     * safe zones.
     *
     * @param solution The previous solution to be used as a starting point for
     * the k-privacy post-processing algorithm.
     *
     * @note This function is called before refining the paths of the agents.
     */
    void initialize_extended_safe_zones_cache(const Solution &solution);

    /**
     * @brief Validates the solution after applying k-privacy post-processing.
     *
     * @note A solution is considered valid if all agents in the solution
     * are in their safe zones at all times, and the solution is feasible for
     * each agent separately.
     *
     * @param ins The instance containing the graph and other parameters.
     * @param solution The solution to be validated.
     * @param solution_found True if a solution is found for all agents, false
     * otherwise.
     * @param based_on_initial_safe_zones True if the solution is based on the
     * initial safe zones, false if based on the extended safe zones.
     * @param verbose Whether to print verbose output (0 for no output, higher
     * values for more output).
     * @return true If the solution is valid, meaning all agents are in their
     * safe zones at all times and the solution is feasible for each agent
     * separately.
     * @return false If the solution is not valid, meaning at least one agent is
     * not in its safe zone at some time step or the solution is not feasible
     * for at least one agent.
     */
    bool validate_k_privacy_post_process_solution(
        const Instance &ins, const Solution &solution, bool solution_found,
        bool based_on_initial_safe_zones, int verbose = 0);

    /**
     * @brief Extends the safe zone of the given agent group at time t to
     * include an additional vertex v if it follows the extension rules.
     *
     * @details The extension rules are as follows:
     * - v is a neighbor of a vertex in the current safe zone at time t.
     * - v is not in the safe zone of any agent group at time t.
     * - v is not in the field of view of any vertex in the safe zone of another
     * agent group at time t.
     * - v is not in the safe zone of any other agent group at time t - 1 and
     * v is not in the initial safe zone of any other agent group at time t + 1.
     * - between the possible vertices that can be added, we add a random one.
     *
     * @param agent_group_id The agent group id.
     * @param t The time step.
     * @return true If a vertex was added to the safe zone.
     * @return false Otherwise.
     */
    bool _extend_safe_zone(int agent_group_id, int t);

    /**
     * @brief Prints the safe zone of the i'th agent group into the ostream
     * given.
     *
     * @param os The ostream to print into.
     * @param i The agent group to print it's safe zone.
     */
    void print_safe_zone(std::ostream &os, int i);

    /**
     * @brief Prints all of the safe zones that are cached into the ostream
     * given.
     *
     * @param os The ostream to print into.
     */
    void print_safe_zones(std::ostream &os);
};

/**
 * @brief A function that calls the KPrivacyPostProcess class to refine the
 * paths of all agents in the solution - making each mock agent as if it is
 * the real agent.
 *
 * @details This function is used for analyzing the quality of the
 * k-privacy post-processing algorithm. It goes through each agent group
 * and refines the path of each agent in the group based on the
 * safe zones defined by the k-privacy post-processing algorithm.
 *
 * @param ins The instance containing the graph and other parameters.
 * @param solution_found Set to true if a solution is found for all agents,
 * false otherwise.
 * @param previous_solution The previous solution to be used as a starting point
 * for the k-privacy post-processing algorithm.
 * @param verbose Whether to print verbose output (0 for no output, higher
 * values for more output).
 * @param deadline The deadline for the algorithm to complete.
 * @param seed The seed for the random number generator, used for
 * reproducibility.
 * @param _kpp Pointer to the returned value of the used KPrivacyPostProcess.
 * @return tuple<Solution, Solution> The refined solutions after applying
 * k-privacy post-processing. The first solution is refined using the initial
 * safe zones, while the second solution is refined using the extended safe
 * zones.
 */
std::tuple<Solution, Solution> solve_with_k_privacy_post_process(
    const Instance &ins, bool *solution_found,
    const Solution &previous_solution, const int verbose = 0,
    const Deadline *deadline = nullptr, int seed = 0,
    KPrivacyPostProcess **_kpp = nullptr);
