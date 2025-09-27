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
      initial_safe_zones_cache(),
      deadline(_deadline),
      extended_safe_zones_list()
{
}

KPrivacyPostProcess::~KPrivacyPostProcess()
{
    for (auto &pair : initial_safe_zones_cache) {
        delete pair.second;  // Clean up allocated safe zones graphs
    }
}

TemporalGraph *KPrivacyPostProcess::_get_initial_safe_zones(int agent_group_id)
{
    auto it = initial_safe_zones_cache.find(agent_group_id);
    if (it != initial_safe_zones_cache.end()) {
        return it->second;  // Return cached safe zones if available
    }
    return nullptr;  // Return nullptr if not cached
}

TemporalGraph *KPrivacyPostProcess::get_initial_safe_zones(
    const Solution &solution, int agent_group_id)
{
    TemporalGraph *safe_zones = _get_initial_safe_zones(agent_group_id);
    if (safe_zones != nullptr) {
        return safe_zones;  // Return cached safe zones if available
    }

    // Create a new TemporalGraph for safe zones if it is not cached.

    safe_zones = new TemporalGraph(ins);

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

    initial_safe_zones_cache[agent_group_id] = safe_zones;
    return safe_zones;
l_cleanup:
    delete safe_zones;  // Clean up if we exit early
    return nullptr;     // Return nullptr if we exit early
}

bool KPrivacyPostProcess::_extend_safe_zone(int agent_group_id, int t)
{
    TemporalGraph *current_safe_zone = extended_safe_zones_list[agent_group_id];
    if (current_safe_zone == nullptr) {
        throw std::runtime_error("Safe zones for agent group " +
                                 std::to_string(agent_group_id) +
                                 " are not cached.");
    }
    // Collect all candidate vertices that can be added to the safe zone at time
    // t
    Vertices candidate_vertices;
    for (TemporalVertex *tv : current_safe_zone->V) {
        bool valid = true;
        // Check if the vertex is a neighbor of any vertex in the safe zone at
        // time t (Rule 1).
        valid = false;
        for (Vertex *neighbor : tv->vertex->neighbor) {
            if (current_safe_zone->V[neighbor->id]->timestamps.count(t) > 0) {
                valid = true;
                break;  // If any neighbor is in the safe zone at time t, add tv
                        // as a candidate
            }
        }
        if (!valid) {
            continue;
        }
        // Check that tv is not in any safe zone at time t (Rule 2).
        for (int other_agent_group_id = 0;
             other_agent_group_id < get_num_of_agent_groups(ins->N, ins->k);
             ++other_agent_group_id) {
            TemporalGraph *other_safe_zone =
                extended_safe_zones_list[other_agent_group_id];
            if (other_safe_zone == nullptr) {
                throw std::runtime_error("Safe zones for agent group " +
                                         std::to_string(other_agent_group_id) +
                                         " are not cached.");
            }
            if (other_safe_zone->V[tv->vertex->id]->timestamps.count(t) > 0) {
                valid =
                    false;  // If tv is in another safe zone at time t, skip it
                break;
            }
            if (other_agent_group_id == agent_group_id) {
                continue;  // Only need to validate that the current safe zone
                           // does not contain tv at time t
            }
        }
        if (!valid) {
            continue;
        }
        // Check that tv is not in the field of view of any vertex in another
        // safe zone at time t (Rule 3).
        for (int other_agent_group_id = 0;
             other_agent_group_id < get_num_of_agent_groups(ins->N, ins->k);
             ++other_agent_group_id) {
            if (other_agent_group_id == agent_group_id) {
                continue;  // Skip checking against its own safe zone
            }
            TemporalGraph *other_safe_zone =
                extended_safe_zones_list[other_agent_group_id];
            if (other_safe_zone == nullptr) {
                throw std::runtime_error("Safe zones for agent group " +
                                         std::to_string(other_agent_group_id) +
                                         " are not cached.");
            }
            if (other_safe_zone->is_in_field_of_view_of_safe_zone(tv->vertex,
                                                                  t)) {
                valid = false;  // If tv is in the field of view of another safe
                                // zone at time t, skip it
                break;
            }
        }
        if (valid) {
            candidate_vertices.push_back(
                tv->vertex);  // Add tv as a candidate vertex
        }
    }
    if (candidate_vertices.empty()) {
        return false;  // No candidates to add
    }
    // Randomly select one candidate vertex to add to the safe zone at time t
    std::uniform_int_distribution<size_t> dist(0,
                                               candidate_vertices.size() - 1);
    size_t selected_index = dist(MT);
    Vertex *selected_vertex = candidate_vertices[selected_index];
    current_safe_zone->add_timestep_to_vertex(selected_vertex, t);
    return true;  // Successfully added a vertex to the safe zone
}

void KPrivacyPostProcess::initialize_extended_safe_zones_cache(
    const Solution &solution)
{
    std::vector<TemporalGraph *> safe_zones_list;
    // Initialize safe zones cache for all agent groups
    for (int agent_group_id = 0;
         agent_group_id < get_num_of_agent_groups(ins->N, ins->k);
         ++agent_group_id) {
        if (is_expired(deadline)) {
            return;  // Stop if the deadline is reached
        }
        TemporalGraph *safe_zones =
            get_initial_safe_zones(solution, agent_group_id);
        safe_zones_list.push_back(safe_zones);  // Populate cache
        extended_safe_zones_list.push_back(
            new TemporalGraph(*safe_zones));  // Create a copy for enhancement
    }
    // Now extend the safe zones by adding random vertices that are not in any
    // safe zone currently.
    for (int t = 0; t < solution.size(); ++t) {
        bool done;
        do {
            done = true;
            for (int agent_group_id = 0;
                 agent_group_id < get_num_of_agent_groups(ins->N, ins->k);
                 ++agent_group_id) {
                if (is_expired(deadline)) {
                    return;  // Stop if the deadline is reached
                }
                bool picked = _extend_safe_zone(agent_group_id, t);
                done = done && !picked;
            }
        } while (!done);
    }
}

Path KPrivacyPostProcess::refine_real_agent_path(TemporalGraph *safe_zones,
                                                 int agent_group_id,
                                                 int real_agent_id)
{
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

TemporalGraph *KPrivacyPostProcess::get_extended_safe_zones(int agent_group_id)
{
    if (extended_safe_zones_list.empty()) {
        throw std::runtime_error(
            "Extended safe zones cache is not initialized. Please call "
            "initialize_extended_safe_zones_cache first.");
    }
    return extended_safe_zones_list[agent_group_id];
}

std::tuple<Solution, Solution> solve_with_k_privacy_post_process(
    const Instance &ins, bool *solution_found,
    const Solution &previous_solution, const int verbose,
    const Deadline *deadline, int seed, KPrivacyPostProcess **_kpp)
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
    Paths IS_refined_paths;
    Paths ES_refined_paths;

    *solution_found = true;  // Indicate that a solution was found

    // Initialize all safe zones cache for all agent groups.
    kpp->initialize_extended_safe_zones_cache(previous_solution);
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

            // Refine path with the initial safe zones
            TemporalGraph *initial_safe_zones =
                kpp->get_initial_safe_zones(previous_solution, agent_group_id);
            if (initial_safe_zones == nullptr) {
                *solution_found = false;
                break;  // Stop if we cannot get the initial safe zones
            }
            Path initial_refined_path = kpp->refine_real_agent_path(
                initial_safe_zones, agent_group_id, real_agent_id);
            // Update the solution with the refined path
            IS_refined_paths.push_back(std::move(initial_refined_path));

            // Refine path with the extended safe zones
            TemporalGraph *extended_safe_zones =
                kpp->get_extended_safe_zones(agent_group_id);
            if (extended_safe_zones == nullptr) {
                *solution_found = false;
                break;  // Stop if we cannot get the initial safe zones
            }
            Path extended_refined_path = kpp->refine_real_agent_path(
                extended_safe_zones, agent_group_id, real_agent_id);
            // Update the solution with the refined path
            ES_refined_paths.push_back(std::move(extended_refined_path));
        }
    }

    if (*solution_found) {
        return std::tuple<Solution, Solution>(from_paths(IS_refined_paths),
                                              from_paths(ES_refined_paths));
    } else {
        // Return an empty solution if not found.
        return std::tuple<Solution, Solution>(Solution(), Solution());
    }
}

bool KPrivacyPostProcess::validate_k_privacy_post_process_solution(
    const Instance &ins, const Solution &solution, bool solution_found,
    bool based_on_initial_safe_zones, const int verbose)
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
            TemporalGraph *safe_zones = nullptr;
            if (based_on_initial_safe_zones) {
                safe_zones =
                    initial_safe_zones_cache[get_agent_group_id(i, ins.k)];
            } else {
                safe_zones =
                    extended_safe_zones_list[get_agent_group_id(i, ins.k)];
            }
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

            // Check for conflicts with other agent groups
            for (size_t j = i + 1; j < config.size(); ++j) {
                size_t agent_group_id_i = get_agent_group_id(i, ins.k);
                size_t agent_group_id_j = get_agent_group_id(j, ins.k);
                if (agent_group_id_i != agent_group_id_j) {
                    // Only if the agent groups are different,
                    // the conflicts should be checked - since agents in the
                    // same group can conflict with each other after the
                    // post-processing algorithm.
                    // Check vertex conflicts
                    if (config[i] == config[j]) {
                        info(0, verbose,
                             "Vertex conflict between agent " +
                                 std::to_string(i) + " and agent " +
                                 std::to_string(j) + " at time step " +
                                 std::to_string(t) + ".");
                        return false;  // Vertex conflict
                    }
                    // Check field of view conflicts
                    if (in_field_of_view(config[i], config[j],
                                         ins.field_of_view_radius)) {
                        info(0, verbose,
                             "Field of view conflict between agent " +
                                 std::to_string(i) + " and agent " +
                                 std::to_string(j) + " at time step " +
                                 std::to_string(t) + ".");
                        return false;  // Field of view conflict
                    }
                }
            }
        }
        if (previous_config != nullptr) {
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
                // Check for conflicts with other agent groups
                for (size_t j = i + 1; j < config.size(); ++j) {
                    size_t agent_group_id_i = get_agent_group_id(i, ins.k);
                    size_t agent_group_id_j = get_agent_group_id(j, ins.k);
                    if (agent_group_id_i != agent_group_id_j) {
                        // Only if the agent groups are different,
                        // the conflicts should be checked - since agents in the
                        // same group can conflict with each other after the
                        // post-processing algorithm.
                        // Check swapping conflicts
                        if (config[i] == previous_config->at(j) &&
                            config[j] == previous_config->at(i)) {
                            info(0, verbose,
                                 "Swapping conflict between agent " +
                                     std::to_string(i) + " and agent " +
                                     std::to_string(j) + " at time step " +
                                     std::to_string(t) + ".");
                            return false;  // Swapping conflict
                        }
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
    os << extended_safe_zones_list[i];
}

void KPrivacyPostProcess::print_safe_zones(std::ostream &os)
{
    for (int i = 0; i < get_num_of_agent_groups(ins->N, ins->k); i++) {
        os << "Safe zone for agent group " << i << ":" << std::endl;
        print_safe_zone(os, i);
    }
}
