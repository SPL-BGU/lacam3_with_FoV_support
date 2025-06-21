#include "../include/post_processing.hpp"

#include "../include/dist_table.hpp"
#include "../include/planner.hpp"

bool is_feasible_solution(const Instance &ins, const Solution &solution,
                          const bool solution_found, const int verbose)
{
    if (!solution_found) {
        return true;
    }
    if (solution.empty()) return true;

    // check start locations
    if (!is_same_config(solution.front(), ins.starts)) {
        info(1, verbose, "invalid starts");
        return false;
    }

    // check goal locations
    if (!is_same_config(solution.back(), ins.goals)) {
        info(1, verbose, "invalid goals");
        return false;
    }

    for (size_t t = 1; t < solution.size(); ++t) {
        for (size_t i = 0; i < ins.N; ++i) {
            auto v_i_from = solution[t - 1][i];
            auto v_i_to = solution[t][i];
            // check connectivity
            if (v_i_from != v_i_to &&
                std::find(v_i_to->neighbor.begin(), v_i_to->neighbor.end(),
                          v_i_from) == v_i_to->neighbor.end()) {
                info(1, verbose, "invalid move");
                return false;
            }

            // check conflicts
            for (size_t j = i + 1; j < ins.N; ++j) {
                auto v_j_from = solution[t - 1][j];
                auto v_j_to = solution[t][j];
                // vertex conflicts
                if (v_j_to == v_i_to) {
                    info(1, verbose, "vertex conflict between agent-", i,
                         " and agent-", j, " at vertex-", v_i_to->id,
                         " at timestep ", t);
                    return false;
                }
                size_t agent_group_id_i = get_agent_group_id(i, ins.k);
                size_t agent_group_id_j = get_agent_group_id(j, ins.k);
                if (agent_group_id_i != agent_group_id_j) {
                    // Only if the agent groups are different, the field of view
                    // should be checked.
                    if (t == 1) {
                        // Check t-1 as well so that the starts will also be
                        // tested:
                        if (in_field_of_view(v_i_from, v_j_from,
                                             ins.field_of_view_radius)) {
                            info(1, verbose,
                                 "field of view conflict at v=" +
                                     std::to_string(v_i_from->id) +
                                     ", u=" + std::to_string(v_j_from->id) +
                                     ", t=" + std::to_string(t - 1));
                            return false;
                        }
                    }
                    if (in_field_of_view(v_i_to, v_j_to,
                                         ins.field_of_view_radius)) {
                        info(1, verbose,
                             "validation, field of view conflict at v=" +
                                 std::to_string(v_i_to->id) +
                                 ", u=" + std::to_string(v_j_to->id) +
                                 ", t=" + std::to_string(t));
                        return false;
                    }
                }
                // swap conflicts
                if (v_j_to == v_i_from && v_j_from == v_i_to) {
                    info(1, verbose, "edge conflict");
                    return false;
                }
            }
        }
    }

    return true;
}

void print_stats(const int verbose, const Deadline *deadline,
                 const Instance &ins, const Solution &solution,
                 const double comp_time_ms)
{
    auto ceil = [](float x) { return std::ceil(x * 100) / 100; };
    auto dist_table = DistTable(ins);
    const auto makespan = get_makespan(solution);
    const auto makespan_lb = get_makespan_lower_bound(ins, dist_table);
    const auto sum_of_costs = get_sum_of_costs(solution);
    const auto sum_of_costs_lb = get_sum_of_costs_lower_bound(ins, dist_table);
    const auto sum_of_loss = get_sum_of_loss(solution);
    info(1, verbose, deadline, "solved", "\tmakespan: ", makespan,
         " (lb=", makespan_lb, ", ub=", ceil((float)makespan / makespan_lb),
         ")", "\tsum_of_costs: ", sum_of_costs, " (lb=", sum_of_costs_lb,
         ", ub=", ceil((float)sum_of_costs / sum_of_costs_lb), ")",
         "\tsum_of_loss: ", sum_of_loss, " (lb=", sum_of_costs_lb,
         ", ub=", ceil((float)sum_of_loss / sum_of_costs_lb), ")");
}

// for log of map_name
static const std::regex r_map_name = std::regex(R"(.+/(.+))");

void make_log(const Instance &ins, const Solution &solution,
              const bool solution_found, const std::string &output_name,
              const double comp_time_ms, const std::string &map_name,
              const int seed, const bool log_short)
{
    // map name
    std::smatch results;
    const auto map_recorded_name =
        (std::regex_match(map_name, results, r_map_name)) ? results[1].str()
                                                          : map_name;

    // for instance-specific values
    auto dist_table = DistTable(ins);
    auto sum_of_costs = solution_found ? get_sum_of_costs(solution) : 0;
    auto makespan = solution_found ? get_makespan(solution) : 0;
    auto sum_of_loss = solution_found ? get_sum_of_loss(solution) : 0;

    // log for visualizer
    auto get_x = [&](int k) { return k % ins.G->width; };
    auto get_y = [&](int k) { return k / ins.G->width; };
    std::ofstream log;
    log.open(output_name, std::ios::out);
    log << "agents=" << ins.N << "\n";
    log << "map_file=" << map_recorded_name << "\n";
    log << "solver=planner\n";
    log << "solved=" << solution_found << "\n";
    log << "soc=" << sum_of_costs << "\n";
    log << "soc_lb=" << get_sum_of_costs_lower_bound(ins, dist_table) << "\n";
    log << "makespan=" << makespan << "\n";
    log << "makespan_lb=" << get_makespan_lower_bound(ins, dist_table) << "\n";
    log << "sum_of_loss=" << sum_of_loss << "\n";
    log << "sum_of_loss_lb=" << get_sum_of_costs_lower_bound(ins, dist_table)
        << "\n";
    log << "comp_time=" << comp_time_ms << "\n";
    log << "seed=" << seed << "\n";
    log << Planner::MSG << "\n";
    if (log_short) return;
    log << "starts=";
    for (size_t i = 0; i < ins.N; ++i) {
        auto k = ins.starts[i]->index;
        log << "(" << get_x(k) << "," << get_y(k) << "),";
    }
    log << "\ngoals=";
    for (size_t i = 0; i < ins.N; ++i) {
        auto k = ins.goals[i]->index;
        log << "(" << get_x(k) << "," << get_y(k) << "),";
    }
    log << "\nsolution=\n";
    for (size_t t = 0; t < solution.size(); ++t) {
        log << t << ":";
        auto C = solution[t];
        for (auto v : C) {
            log << "(" << get_x(v->index) << "," << get_y(v->index) << "),";
        }
        log << "\n";
    }
    log.close();
}
