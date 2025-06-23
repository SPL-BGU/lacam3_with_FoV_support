#pragma once

#include "dist_table.hpp"
#include "graph.hpp"
#include "instance.hpp"
#include "k_privacy_post_process.hpp"
#include "lacam.hpp"
#include "pibt.hpp"
#include "pibt_solve.hpp"
#include "post_processing.hpp"
#include "sipp.hpp"
#include "utils.hpp"

enum class PlannerType { PIBT, LaCAM, KPrivacyPostProcess };

static std::unordered_map<std::string, PlannerType> const planner_type_map = {
    {"pibt", PlannerType::PIBT},
    {"lacam", PlannerType::LaCAM},
    {"k_privacy_post_process", PlannerType::KPrivacyPostProcess}};

inline PlannerType get_planner_type(const std::string &planner_type_str)
{
    auto it = planner_type_map.find(planner_type_str);
    if (it != planner_type_map.end()) {
        return it->second;
    } else {
        throw std::invalid_argument("Unknown planner type: " +
                                    planner_type_str);
    }
}

Solution solve_with_planner(const Instance &ins, const PlannerType planner_type,
                            bool *solution_found, int verbose = 0,
                            const Deadline *deadline = nullptr, int seed = 0,
                            int max_iter_count = 1000,
                            const Solution *previous_solution = nullptr,
                            KPrivacyPostProcess **kpp = nullptr);
