#include "../include/pibt_solve.hpp"

#include "../include/planner.hpp"

Solution pibt_main_loop(PIBT &pibt, bool *solution_found, int verbose,
                        const Deadline *deadline, int max_iter_count);
void logging(double time_solution, int search_iteration);

Solution pibt_solve(const Instance &ins, bool *solution_found, int verbose,
                    const Deadline *deadline, int seed, int max_iter_count)
{
    // PIBT(const Instance *_ins, DistTable *_D, int seed = 0,
    //  bool _flg_swap = true, Scatter *_scatter = nullptr);
    info(1, verbose, deadline, "pre-processing");
    // Initialize a new Planner for getting the scatter from it.
    auto planner = Planner(&ins, verbose, deadline, seed);
    planner.set_scatter();
    auto pibt =
        PIBT(planner.ins, planner.D, seed, planner.FLG_SWAP, planner.scatter);
    return pibt_main_loop(pibt, solution_found, verbose, deadline,
                          max_iter_count);
}

Solution pibt_main_loop(PIBT &pibt, bool *solution_found, int verbose,
                        const Deadline *deadline, int max_iter_count)
{
    info(1, verbose, deadline, "start search");
    Solution solution;
    Config current_config = pibt.ins->starts;
    PIBTAgents agents(pibt.N);
    std::vector<int> order(pibt.N);
    double time_solution = -1;
    int t = 0;
    *solution_found = false;

    for (int i = 0; i < pibt.N; ++i) {
        agents[i].elapsed = 0;
        agents[i].init_distance = pibt.D->get(i, current_config[i]);
        agents[i].tie_breaker = get_random_float(pibt.MT);
    }

    std::iota(order.begin(), order.end(), 0);

    // First config in solution.
    solution.push_back(current_config);

    for (t = 0; t < max_iter_count; t++) {
        info(2, verbose, deadline, "search iteration: ", t);
        Config next_config = Config(pibt.N, nullptr);
        if (is_expired(deadline)) {
            info(1, verbose, deadline, "deadline expired at timestamp: ", t);
            break;
        }
        // Sort indices based on agents' comparison criteria.
        // The order is reversed since the priority is given to agents with
        // higher elapsed time, higher initial distance, and higher tie breaker
        // value.
        std::sort(order.begin(), order.end(),
                  [&](int a, int b) { return agents[a] > agents[b]; });
        if (!pibt.set_new_config(current_config, next_config, order)) {
            std::cerr << "Failed to set new configuration." << std::endl;
            exit(1);
        }
        solution.push_back(next_config);

        // Update agents' elapsed time.
        for (int i = 0; i < pibt.N; ++i) {
            // Reset elapsed (priority) if at goal.
            agents[i].elapsed = (pibt.ins->goals[i]->id == next_config[i]->id)
                                    ? 0
                                    : agents[i].elapsed + 1;
        }

        // Check goal condition.
        if (is_same_config(next_config, pibt.ins->goals)) {
            time_solution = deadline->elapsed_ms();
            *solution_found = true;

            info(1, verbose, deadline, "found solution at timestamp: ", t);
            // PIBT is not designed to continue after finding a solution,
            // so finish the search.
            break;
        }
        if (is_same_config(next_config, current_config)) {
            info(1, verbose, deadline, "no progress at timestamp: ", t);
            break;  // No progress, exit the loop.
        }
        // Update current config for the next iteration.
        current_config = next_config;
    }
    logging(time_solution, t);
    return solution;
}

void logging(double time_solution, int search_iteration)
{
    Planner::MSG += "\ncomp_time_solution=" + std::to_string(time_solution);
    Planner::MSG += "\nsearch_iteration=" + std::to_string(search_iteration);
}
