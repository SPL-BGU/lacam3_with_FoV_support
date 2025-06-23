#include "../include/planner_choice.hpp"

Solution solve_with_planner(const Instance &ins, const PlannerType planner_type,
                            bool *solution_found, int verbose,
                            const Deadline *deadline, int seed,
                            int max_iter_count,
                            const Solution *previous_solution,
                            KPrivacyPostProcess **kpp)
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
    } else if (planner_type == PlannerType::KPrivacyPostProcess) {
        if (nullptr == previous_solution) {
            throw std::invalid_argument(
                "Previous solution is required for k-privacy post-processing");
        }
        return solve_with_k_privacy_post_process(ins, solution_found,
                                                 *previous_solution, verbose,
                                                 deadline, seed, kpp);
    } else {
        throw std::invalid_argument("Unknown planner type");
    }
}
