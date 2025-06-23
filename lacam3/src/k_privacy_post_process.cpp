#include "../include/k_privacy_post_process.hpp"

#include "../include/field_of_view.hpp"
#include "../include/metrics.hpp"

KPrivacyPostProcess::KPrivacyPostProcess(const Instance *instance,
                                         DistTable *_D, int seed, int verbosity,
                                         const Deadline *_deadline)
    : ins(instance),
      D(_D),
      verbose(verbosity),
      MT(seed),
      safe_zones_cache(),
      deadline(_deadline),
      enhanced_safe_zones_list()
{
}

KPrivacyPostProcess::~KPrivacyPostProcess()
{
    for (auto &pair : safe_zones_cache) {
        delete pair.second;  // Clean up allocated safe zones graphs
    }
}

TemporalGraph *KPrivacyPostProcess::_get_safe_zones(int agent_group_id)
{
    auto it = safe_zones_cache.find(agent_group_id);
    if (it != safe_zones_cache.end()) {
        return it->second;  // Return cached safe zones if available
    }
    return nullptr;  // Return nullptr if not cached
}

TemporalGraph *KPrivacyPostProcess::get_safe_zones(const Solution &solution,
                                                   int agent_group_id)
{
    TemporalGraph *safe_zones = _get_safe_zones(agent_group_id);
    if (safe_zones != nullptr) {
        return safe_zones;  // Return cached safe zones if available
    }

    // Create a new TemporalGraph for safe zones if it is not cached.

    safe_zones = new TemporalGraph(ins->G);

    for (size_t t = 0; t < solution.size(); ++t) {
        if (is_expired(deadline)) {
            goto l_cleanup;  // Stop if the deadline is reached
        }
        const Config &config = solution[t];
        // Will hold all vertices in the field of view of the group in the
        // current timestamp as they are candidates for the safe zones.
        std::unordered_set<Vertex *> current_field_of_view;
        for (size_t j = 0; j < config.size(); ++j) {
            if (is_expired(deadline)) {
                goto l_cleanup;  // Stop if the deadline is reached
            }
            if (get_agent_group_id(j, ins->k) == agent_group_id) {
                // Get the field of view for the agent in the current timestamp
                Vertices field_of_view = get_field_of_view(
                    ins->G, config[j], ins->field_of_view_radius);

                current_field_of_view.insert(field_of_view.begin(),
                                             field_of_view.end());
            }
        }
        // Now we have all vertices in the field of view of the group in the
        // current timestamp, we can check if they are safe.
        // Iterate over each vertex in the current field of view and check if
        // its entire field of view is inside the safe field of view of the
        // group.
        for (Vertex *v : current_field_of_view) {
            if (is_expired(deadline)) {
                goto l_cleanup;  // Stop if the deadline is reached
            }
            Vertices v_field_of_view =
                get_field_of_view(ins->G, v, ins->field_of_view_radius);
            // Check that the entire field of view of the vertex is inside the
            // safe field of view of the group.
            bool is_safe = true;
            for (Vertex *fv : v_field_of_view) {
                if (is_expired(deadline)) {
                    goto l_cleanup;  // Stop if the deadline is reached
                }
                // Check if the vertex is in the safe zone
                if (current_field_of_view.find(fv) ==
                    current_field_of_view.end()) {
                    is_safe = false;
                    break;  // If any vertex in the field of view is not in the
                            // safe zone, break
                }
            }
            if (is_safe) {
                // Add the vertex to the safe zones for the current timestamp
                safe_zones->add_timestep_to_vertex(v, t);
            }
        }
    }

    safe_zones_cache[agent_group_id] = safe_zones;
    return safe_zones;
l_cleanup:
    delete safe_zones;  // Clean up if we exit early
    return nullptr;     // Return nullptr if we exit early
}

void KPrivacyPostProcess::initialize_safe_zones_cache(const Solution &solution)
{
    std::vector<TemporalGraph *> safe_zones_list;
    // Initialize safe zones cache for all agent groups
    for (int agent_group_id = 0;
         agent_group_id < get_num_of_agent_groups(ins->N, ins->k);
         ++agent_group_id) {
        if (is_expired(deadline)) {
            return;  // Stop if the deadline is reached
        }
        TemporalGraph *safe_zones = get_safe_zones(solution, agent_group_id);
        safe_zones_list.push_back(safe_zones);  // Populate cache
        enhanced_safe_zones_list.push_back(
            new TemporalGraph(*safe_zones));  // Create a copy for enhancement
    }
    // Now extend the safe zones to closest locations which are not safe for any
    // agent group.
    // Goes over all safe zones and each vertex that is not safe for any
    // agent group is defined safe for the closest agent group.
    // If the vertex's distance from the closet two agent groups (d1 and d2) is
    // `|d1 - d2| <= 1.5` (For enabling distance also in diagonal moves), or it
    // is in the field of view of two agent groups - it is not safe for any
    // agent group, for being fair to all agent groups.
    for (int t = 0; t < solution.size(); ++t) {
        for (int id = 0; id < ins->G->V.size(); ++id) {
            if (is_expired(deadline)) {
                return;  // Stop if the deadline is reached
            }
            Vertex *v = ins->G->V[id];
            double min_distance = std::numeric_limits<double>::max();
            int closest_agent_group_id = -1;
            double second_min_distance = std::numeric_limits<double>::max();
            int second_closest_agent_group_id = -1;
            bool safe_for_any_agent_group = false;

            for (int i = 0; i < safe_zones_list.size(); ++i) {
                double distance =
                    safe_zones_list[i]->distance_from_safe_zone(v, t);
                if (0 == distance) {
                    // If the vertex is safe for this agent group, skip it
                    safe_for_any_agent_group = true;
                    break;
                }
                if (distance < min_distance) {
                    second_min_distance = min_distance;
                    second_closest_agent_group_id = closest_agent_group_id;
                    min_distance = distance;
                    closest_agent_group_id = i;
                } else if (distance < second_min_distance) {
                    second_min_distance = distance;
                    second_closest_agent_group_id = i;
                }
            }
            if (safe_for_any_agent_group) {
                continue;  // If the vertex is safe for any agent group, skip it
            }
            // If the vertex is not safe for any agent group, check the distance
            // from the closest two agent groups.
            if (closest_agent_group_id != -1) {
                if (second_closest_agent_group_id != -1) {
                    double distance_diff =
                        std::abs(min_distance - second_min_distance);
                    if (distance_diff <= 1.5) {
                        // If the distance difference is within the threshold,
                        // it is not safe for any agent group.
                        continue;
                    }
                }
                // If we reached here, the vertex is safe for the closest agent
                // group - add it to the enhanced safe zone, so it will not be
                // calculated for other vertices in this timestamp.
                enhanced_safe_zones_list[closest_agent_group_id]
                    ->add_timestep_to_vertex(v, t);
            }
        }
    }
}

Path KPrivacyPostProcess::refine_real_agent_path(const Solution &solution,
                                                 int agent_group_id,
                                                 int real_agent_id)
{
    // Get the safe zones for the agent's group
    // TemporalGraph *safe_zones = get_safe_zones(solution, agent_group_id);
    TemporalGraph *safe_zones =
        enhanced_safe_zones_list[agent_group_id];  // Use enhanced safe zones
    if (is_expired(deadline)) {
        return Path();  // Return empty path if deadline is reached
    }

    // Convert to safe intervals
    SITable safe_intervals = safe_zones->to_safe_interval_table();

    TemporalGraphConflictChecker conflict_checker(safe_zones);

    auto real_agent_absolute_id = real_agent_id + agent_group_id * ins->k;

    // Run SIPP to find the refined path for the real agent
    Vertex *start_vertex = ins->starts[real_agent_absolute_id];
    Vertex *goal_vertex = ins->goals[real_agent_absolute_id];
    Path refined_path =
        sipp(real_agent_absolute_id, start_vertex, goal_vertex, D, nullptr,
             deadline, INT_MAX, &conflict_checker, &safe_intervals);

    return refined_path;  // Return the refined path for the real agent
}

Solution solve_with_k_privacy_post_process(const Instance &ins,
                                           bool *solution_found,
                                           const Solution &previous_solution,
                                           const int verbose,
                                           const Deadline *deadline, int seed,
                                           KPrivacyPostProcess **_kpp)
{
    DistTable D(&ins);
    KPrivacyPostProcess *kpp =
        new KPrivacyPostProcess(&ins, &D, seed, verbose, deadline);
    if (_kpp != nullptr) {
        *_kpp = kpp;  // Set the pointer to the KPrivacyPostProcess object
    } else {
        throw std::runtime_error(
            "KPrivacyPostProcess pointer is null, cannot proceed with "
            "post-processing.");
    }
    Paths refined_paths;

    std::cout << "Previous solution's cost = "
              << get_sum_of_costs(previous_solution) << std::endl;

    *solution_found = true;  // Indicate that a solution was found

    // Initialize all safe zones cache for all agent groups.
    kpp->initialize_safe_zones_cache(previous_solution);
    // Iterate over each agent group
    for (int agent_group_id = 0;
         agent_group_id < get_num_of_agent_groups(ins.N, ins.k);
         ++agent_group_id) {
        if (is_expired(deadline)) {
            *solution_found = false;
            break;  // Stop if the deadline is reached
        }
        for (int real_agent_id = 0; real_agent_id < ins.k; ++real_agent_id) {
            if (is_expired(deadline)) {
                *solution_found = false;
                break;  // Stop if the deadline is reached
            }
            Path refined_path = kpp->refine_real_agent_path(
                previous_solution, agent_group_id, real_agent_id);
            // Update the solution with the refined path
            refined_paths.push_back(std::move(refined_path));
        }
    }

    if (*solution_found) {
        std::cout << "Refined solution's cost = "
                  << get_sum_of_costs_paths(refined_paths) << std::endl;
        return from_paths(refined_paths);
    } else {
        // Return an empty solution if not found.
        return Solution();
    }
}

bool KPrivacyPostProcess::validate_k_privacy_post_process_solution(
    const Instance &ins, const Solution &solution, bool solution_found,
    const int verbose)
{
    if (!solution_found) {
        return true;  // If no solution is found, we consider it valid
    }
    if (solution.empty()) {
        return true;  // If the solution is empty, we consider it valid
    }

    const Config *previous_config = nullptr;

    for (int t = 0; t < solution.size(); ++t) {
        const Config &config = solution[t];
        if (config.size() != ins.N) {
            info(0, verbose, "Config size does not match number of agents.");
            return false;
        }
        // Check if the agent is in its safe zone at this time step
        for (size_t i = 0; i < config.size(); ++i) {
            TemporalGraph *safe_zones =
                enhanced_safe_zones_list[get_agent_group_id(i, ins.k)];
            if (safe_zones == nullptr) {
                throw std::runtime_error(
                    "Safe zones for agent group " +
                    std::to_string(get_agent_group_id(i, ins.k)) +
                    " are not cached.");
            }
            TemporalVertex *v = safe_zones->V[config[i]->id];
            if (v->timestamps.find(t) == v->timestamps.end()) {
                info(0, verbose,
                     "Agent " + std::to_string(i) +
                         " is not in its safe zone at time step " +
                         std::to_string(t) + ".");
                return false;  // Agent is not in its safe zone at this time
                               // step
            }
        }
        if (previous_config != nullptr) {
            // Check if the current configuration is different from the
            // previous one
            if (*previous_config == config) {
                info(0, verbose, "Consecutive configurations are the same.");
                return false;
            }
            for (size_t i = 0; i < config.size(); ++i) {
                // Check connectivity between the current and previous
                // configurations
                // Allow stay in place actions:
                if (config[i]->id != previous_config->at(i)->id) {
                    // If not a stay in place action - check that the new
                    // vertex is a neighbor of the previous one.
                    if (find(previous_config->at(i)->neighbor.begin(),
                             previous_config->at(i)->neighbor.end(),
                             config[i]) ==
                        previous_config->at(i)->neighbor.end()) {
                        info(0, verbose,
                             "Agent " + std::to_string(i) +
                                 " is not moving to a neighbor at time "
                                 "step " +
                                 std::to_string(t) + ".");
                        return false;  // Agent is not moving to a neighbor
                    }
                }
            }
        }
        previous_config = &config;
    }

    return true;  // The solution is valid
}

void KPrivacyPostProcess::print_safe_zone(std::ostream &os, int i)
{
    os << enhanced_safe_zones_list[i];
}

void KPrivacyPostProcess::print_safe_zones(std::ostream &os)
{
    for (int i = 0; i < get_num_of_agent_groups(ins->N, ins->k); i++) {
        os << "Safe zone for agent group " << i << ":" << std::endl;
        print_safe_zone(os, i);
    }
}
