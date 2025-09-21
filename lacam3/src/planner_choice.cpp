#include "../include/planner_choice.hpp"

Solution solve_with_planner(const Instance &ins, const PlannerType planner_type,
                            bool *solution_found, int verbose,
                            const Deadline *deadline, int seed,
                            int max_iter_count)
{
    if (planner_type == PlannerType::PIBT) {
        return pibt_solve(ins, solution_found, verbose, deadline, seed,
                          max_iter_count);
    } else if (planner_type == PlannerType::LaCAM) {
        Solution result = solve(ins, verbose, deadline, seed);
        if (result.empty()) {
            *solution_found = false;
        } else {
            *solution_found = true;
        }
        return result;
    } else {
        throw std::invalid_argument("Unknown planner type");
    }
}
