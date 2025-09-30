#include "../include/k_privacy_post_process.hpp"

#include <chrono>

#include "../include/field_of_view.hpp"
#include "../include/metrics.hpp"

static int64_t total_extension_time_microseconds;

KPrivacyPostProcess::KPrivacyPostProcess(const Instance *instance,
                                         DistTable *_D, int max_timestamp,
                                         int seed, int verbosity,
                                         const Deadline *_deadline)
    : ins(instance),
      D(_D),
      verbose(verbosity),
      MT(seed),
      initial_safe_zones_cache(),
      deadline(_deadline),
      extended_safe_zones_list(),
      timestamp__agent_group__possible_extend_vertices(
          max_timestamp + 1,
          std::vector<RandomizedSet>(get_num_of_agent_groups(ins->N, ins->k))),
      timestamp__vertex__agent_groups(
          max_timestamp + 1, std::vector<std::vector<int>>(ins->G->V.size()))
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

    safe_zones = new TemporalGraph(ins, solution.size() - 1);

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

void KPrivacyPostProcess::
    _initialize_current_agent_group_possible_extend_vertices(int t)
{
    for (int agent_group_id = 0;
         agent_group_id < get_num_of_agent_groups(ins->N, ins->k);
         ++agent_group_id) {
        _initialize_agent_group_possible_vertices(agent_group_id, t);
    }
}

void KPrivacyPostProcess::_check_and_add_vertex_to_possible_extend(
    Vertex *v, int agent_group_id, int t)
{
    TemporalGraph *current_safe_zone = extended_safe_zones_list[agent_group_id];
    if (current_safe_zone == nullptr) {
        throw std::runtime_error("Safe zones for agent group " +
                                 std::to_string(agent_group_id) +
                                 " are not cached.");
    }
    bool valid = true;
    // Check if the vertex is a neighbor of any vertex in the safe zone at
    // time t (Rule 1).
    valid = false;
    for (Vertex *neighbor : v->neighbor) {
        if (current_safe_zone->timestamp_to_vertices_map[t][neighbor->id]) {
            valid = true;
            break;  // If any neighbor is in the safe zone at time t, add tv
                    // as a candidate
        }
    }
    if (!valid) {
        return;
    }
    // Check that v is not in any safe zone at time t (Rule 2).
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
        if (other_safe_zone->timestamp_to_vertices_map[t][v->id]) {
            valid = false;  // If v is in another safe zone at time t, skip it
            break;
        }
    }
    if (!valid) {
        return;
    }
    auto start = std::chrono::high_resolution_clock::now();
    // Check that v is not in the field of view of any vertex in another
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
        if (other_safe_zone->is_in_field_of_view_of_safe_zone(v, t)) {
            valid = false;  // If v is in the field of view of another safe
                            // zone at time t, skip it
            break;
        }
    }
    auto end = std::chrono::high_resolution_clock::now();
    total_extension_time_microseconds +=
        std::chrono::duration_cast<std::chrono::microseconds>(end - start)
            .count();
    if (valid) {
        bool inserted =
            timestamp__agent_group__possible_extend_vertices[t][agent_group_id]
                .insert(v->id);
        if (inserted) {
            timestamp__vertex__agent_groups[t][v->id].push_back(agent_group_id);
        }
    }
}

void KPrivacyPostProcess::_initialize_agent_group_possible_vertices(
    int agent_group_id, int t)
{
    TemporalGraph *current_safe_zone = extended_safe_zones_list[agent_group_id];
    if (current_safe_zone == nullptr) {
        throw std::runtime_error("Safe zones for agent group " +
                                 std::to_string(agent_group_id) +
                                 " are not cached.");
    }
    // Collect all candidate vertices that can be added to the safe zone at time
    // t
    for (TemporalVertex *tv : current_safe_zone->V) {
        _check_and_add_vertex_to_possible_extend(tv->vertex, agent_group_id, t);
    }
}

void KPrivacyPostProcess::_add_vertex_to_ES(Vertex *v, int agent_group_id,
                                            int t)
{
    TemporalGraph *current_safe_zone = extended_safe_zones_list[agent_group_id];
    if (current_safe_zone == nullptr) {
        throw std::runtime_error("Safe zones for agent group " +
                                 std::to_string(agent_group_id) +
                                 " are not cached.");
    }
    current_safe_zone->add_timestep_to_vertex(v, t, false);
    // Remove v and it's field of view from the possible extend vertices of all
    // agent groups that have it
    Vertices v_field_of_view =
        get_field_of_view(ins->G, v, ins->field_of_view_radius);
    for (Vertex *fv : v_field_of_view) {
        for (int ag_id : timestamp__vertex__agent_groups[t][fv->id]) {
            timestamp__agent_group__possible_extend_vertices[t][ag_id].remove(
                fv->id);
        }
        timestamp__vertex__agent_groups[t][fv->id].clear();
    }
    Vertices v_neighbors = v->neighbor;
    for (Vertex *neighbor : v_neighbors) {
        _check_and_add_vertex_to_possible_extend(neighbor, agent_group_id, t);
    }
}

bool KPrivacyPostProcess::_choose_random_vertex_and_add_to_ES(
    int agent_group_id, int t)
{
    int vertex_id = -1;
    bool had_vertex =
        timestamp__agent_group__possible_extend_vertices[t][agent_group_id]
            .getRandom(MT, &vertex_id);
    if (!had_vertex) {
        return false;  // No candidates to add
    }
    Vertex *selected_vertex = ins->G->V[vertex_id];
    _add_vertex_to_ES(selected_vertex, agent_group_id, t);
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
        std::cout << "Extending safe zones at time " << t << "..." << std::endl;
        _initialize_current_agent_group_possible_extend_vertices(t);
        std::cout << "Initialized groups" << std::endl;
        int loop_idx = 0;
        total_extension_time_microseconds = 0;
        bool done;
        do {
            if (loop_idx % 1000 == 0) {
                std::cout << "Extension loop iteration " << loop_idx
                          << " at time " << t << "..." << std::endl;
            }
            loop_idx++;
            done = true;
            for (int agent_group_id = 0;
                 agent_group_id < get_num_of_agent_groups(ins->N, ins->k);
                 ++agent_group_id) {
                if (is_expired(deadline)) {
                    return;  // Stop if the deadline is reached
                }
                bool picked =
                    _choose_random_vertex_and_add_to_ES(agent_group_id, t);
                done = done && !picked;
            }
        } while (!done);
        double average_extension_time =
            static_cast<double>(total_extension_time_microseconds) /
            static_cast<double>(loop_idx);
        std::cout << "Finished extending safe zones after " << loop_idx
                  << " iterations at time " << t
                  << " --- avg extension session = " << average_extension_time
                  << " microseconds." << std::endl;
    }
    for (TemporalGraph *tg : extended_safe_zones_list) {
        tg->complete_temporal_vertex_timestamps();
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
    KPrivacyPostProcess *kpp = new KPrivacyPostProcess(
        &ins, &D, previous_solution.size() - 1, seed, verbose, deadline);
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
            if (!safe_zones->timestamp_to_vertices_map[t][config[i]->id]) {
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
