/*
 * instance definition
 */
#pragma once
#include <random>

#include "graph.hpp"
#include "utils.hpp"

struct Instance {
    Graph *G;       // graph
    Config starts;  // initial configuration
    Config goals;   // goal configuration
    const uint N;   // number of agents
    /**
     * @brief The radius of the field of view of agents.
     *
     * @note Each agent must keep away from other agents by at least this
     * radius.
     */
    int field_of_view_radius;
    /**
     * @brief The amount of mock agents used for each agent.
     *
     */
    size_t k;
    bool delete_graph_after_used;

    Instance(Graph *_G, const Config &_starts, const Config &_goals, uint _N,
             int _field_of_view_radius = 0, size_t _k = 1);
    Instance(const std::string &map_filename,
             const std::vector<int> &start_indexes,
             const std::vector<int> &goal_indexes,
             int _field_of_view_radius = 0, size_t _k = 1);
    // for MAPF benchmark
    Instance(const std::string &scen_filename, const std::string &map_filename,
             const int _N = 1, int _field_of_view_radius = 0, size_t _k = 1);
    // random instance generation
    Instance(const std::string &map_filename, const int _N = 1,
             const int seed = 0, int _field_of_view_radius = 0, size_t _k = 1);
    ~Instance();

    // simple feasibility check of instance
    bool is_valid(const int verbose = 0) const;
};

// solution: a sequence of configurations
using Solution = std::vector<Config>;

/**
 * @brief Loads a solution from a file.
 *
 * @param ins The instance containing the graph and other parameters.
 * @param solution The solution to be loaded.
 * @param solution_file The file containing the solution.
 * @throws std::runtime_error if the file cannot be opened or the format is
 * invalid.
 */
void load_solution(const Instance &ins, Solution &solution,
                   const std::string &solution_file);

/**
 * @brief Create a Solution from a collection of paths.
 *
 * @note If a path ends before other paths, fills the rest of it in the
 * solution's next configs with the last vertex of the path (the goal).
 *
 * @param paths The paths to create the solution from.
 * @return Solution The generated solution.
 */
Solution from_paths(const Paths paths);
