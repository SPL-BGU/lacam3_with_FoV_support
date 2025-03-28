#include "../include/pibt.hpp"

PIBT::PIBT(const Instance *_ins, DistTable *_D, int seed, bool _flg_swap,
           Scatter *_scatter)
    : ins(_ins),
      MT(std::mt19937(seed)),
      N(ins->N),
      V_size(ins->G->size()),
      D(_D),
      NO_AGENT(N),
      occupied_now(V_size, NO_AGENT),
      occupied_next(V_size, NO_AGENT),
      occupied_field_of_view_now(Fields()),
      occupied_field_of_view_next(Fields()),
      C_next(N, std::array<Vertex *, 5>()),
      tie_breakers(V_size, 0),
      flg_swap(_flg_swap),
      scatter(_scatter)
{
  size_t count = get_num_of_agent_groups(N, ins->k);
  for (size_t i = 0; i < V_size; ++i) {
    occupied_field_of_view_now.push_back(CounterWithSize(count));
    occupied_field_of_view_next.push_back(CounterWithSize(count));
  }
}

PIBT::~PIBT() {}

bool PIBT::can_occupy(std::vector<int> &occupied,
                      Fields &occupied_field_of_view, Vertex *node, int agent)
{
  if (NO_AGENT == agent) {
    throw std::runtime_error("Agent must exist.\n");
  }
  // Check if node already occupied
  if (NO_AGENT != occupied[node->id]) {
    return false;
  }

  size_t group_id = get_agent_group_id(agent, ins->k);

  // Check if node is in the field of view of another agent's group
  size_t other_groups_amount =
      occupied_field_of_view[node->id].size() -
      occupied_field_of_view[node->id].get_agent_group_count(group_id);
  return other_groups_amount == 0;
}

void PIBT::occupy(std::vector<int> &occupied, Fields &occupied_field_of_view,
                  Vertex *node, int agent)
{
  if (NO_AGENT == agent) {
    throw std::runtime_error("Agent must exist.\n");
  }
  if (NO_AGENT != occupied[node->id]) {
    throw std::runtime_error("The node is already occupied.\n");
  }
  if (!can_occupy(occupied, occupied_field_of_view, node, agent)) {
    throw std::runtime_error("The node cannot be occupied.\n");
  }

  Vertices field_of_view =
      get_field_of_view(ins->G, node, ins->field_of_view_radius);
  size_t group_id = get_agent_group_id(agent, ins->k);

  for (const Vertex *current_node : field_of_view) {
    occupied_field_of_view[current_node->id].occupy(agent, group_id);
  }
  occupied[node->id] = agent;
}

void PIBT::deoccupy(std::vector<int> &occupied, Fields &occupied_field_of_view,
                    Vertex *node, int agent)
{
  if (NO_AGENT == agent) {
    throw std::runtime_error("Agent must exist.\n");
  }
  if (agent != occupied[node->id]) {
    throw std::runtime_error("The node is not occupied by the given agent.\n");
  }

  Vertices field_of_view =
      get_field_of_view(ins->G, node, ins->field_of_view_radius);
  size_t group_id = get_agent_group_id(agent, ins->k);

  for (const Vertex *current_node : field_of_view) {
    if (occupied_field_of_view[current_node->id].is_occupied_by_agent(agent)) {
      occupied_field_of_view[current_node->id].deoccupy(agent, group_id);
    } else {
      throw std::runtime_error(
          "The node is not occupied by the given agent.\n");
    }
  }

  occupied[node->id] = NO_AGENT;
}

bool PIBT::_set_new_config(const Config &Q_from, Config &Q_to,
                           const std::vector<int> &order)
{
  bool success = true;
  bool first_agent = true;
  // setup cache & constraints check
  for (auto i = 0; i < N; ++i) {
    // set occupied now
    if (can_occupy(occupied_now, occupied_field_of_view_now, Q_from[i], i)) {
      occupy(occupied_now, occupied_field_of_view_now, Q_from[i], i);
    } else {
      for (auto j = 0; j < i; j++) {
        std::cout << "Agent " << j << " is at (" << Q_from[j]->x << ","
                  << Q_from[j]->y << ")" << std::endl;
      }
      throw std::runtime_error(
          "Failed in initialization - the node (" +
          std::to_string(Q_from[i]->x) + "," + std::to_string(Q_from[i]->y) +
          ") cannot be occupied by agent " + std::to_string(i));
    }

    // set occupied next
    if (Q_to[i] != nullptr) {
      // vertex collision --- not really necessary, since it is included in the
      // can_occupy function, keeping it for readability
      if (occupied_next[Q_to[i]->id] != NO_AGENT) {
        success = false;
        break;
      }
      // swap collision
      auto j = occupied_now[Q_to[i]->id];
      if (j != NO_AGENT && j != i && Q_to[j] == Q_from[i]) {
        success = false;
        break;
      }
      // field of view collision
      if (!can_occupy(occupied_next, occupied_field_of_view_next, Q_to[i], i)) {
        success = false;
        break;
      }
      occupy(occupied_next, occupied_field_of_view_next, Q_to[i], i);
    }
  }

  if (success) {
    for (auto i : order) {
      if (Q_to[i] == nullptr) {
        std::set<size_t> affected_agents = funcPIBT(i, Q_from, Q_to);
        if (affected_agents.empty()) {
          if (first_agent) {  // if the first agent fails, no need to continue -
                              // the pibt must allow the highest priority agent
                              // to move.
            success = false;
            break;
          }

          if (nullptr != Q_to[i]) {
            throw std::runtime_error(
                "The agent " + std::to_string(i) +
                " has a next location, but the affected agents are empty.");
          }
          if (!can_occupy(occupied_next, occupied_field_of_view_next, Q_from[i],
                          i)) {
            std::cout << "Agent " << i
                      << " cannot occupy its current location (" << Q_from[i]->x
                      << "," << Q_from[i]->y << ")" << std::endl;
            std::cout << "The following agents are at the following locations:"
                      << std::endl;
            for (auto j = 0; j < N; j++) {
              std::cout << "Agent " << j << " is at (" << Q_from[j]->x << ","
                        << Q_from[j]->y << ")" << std::endl;
            }
            throw std::runtime_error("The agent " + std::to_string(i) +
                                     " cannot occupy its current location.");
          }
          // Failed to plan the next location for agent i, keep it in it's
          // current location
          occupy(occupied_next, occupied_field_of_view_next, Q_from[i], i);
          Q_to[i] = Q_from[i];
        }
      }
      first_agent = false;
    }
  }

  // cleanup
  for (auto i = 0; i < N; ++i) {
    if (occupied_now[Q_from[i]->id] != NO_AGENT) {
      deoccupy(occupied_now, occupied_field_of_view_now, Q_from[i], i);
    }
    if (Q_to[i] != nullptr && occupied_next[Q_to[i]->id] != NO_AGENT) {
      deoccupy(occupied_next, occupied_field_of_view_next, Q_to[i], i);
    }
  }

  for (auto i = 0; i < N; ++i) {
    // set occupied now
    if (can_occupy(occupied_next, occupied_field_of_view_next, Q_to[i], i)) {
      occupy(occupied_next, occupied_field_of_view_next, Q_to[i], i);
    } else {
      for (auto j = 0; j < i; j++) {
        std::cout << "Agent " << j << " is at (" << Q_to[j]->x << ","
                  << Q_to[j]->y << ")" << std::endl;
      }
      throw std::runtime_error(
          "Failed in Q_to - the node (" + std::to_string(Q_to[i]->x) + "," +
          std::to_string(Q_to[i]->y) + ") cannot be occupied by agent " +
          std::to_string(i));
    }
  }

  // cleanup
  for (auto i = 0; i < N; ++i) {
    if (Q_to[i] != nullptr && occupied_next[Q_to[i]->id] != NO_AGENT) {
      deoccupy(occupied_next, occupied_field_of_view_next, Q_to[i], i);
    }
  }

  return success;
}

bool PIBT::set_new_config(const Config &Q_from, Config &Q_to,
                          const std::vector<int> &order)
{
  try {
    return _set_new_config(Q_from, Q_to, order);
  } catch (const std::exception &e) {
    std::cerr << "Error in PIBT: " << e.what() << std::endl;
    exit(1);
  }
}

std::set<size_t> PIBT::funcPIBT(const int i, const Config &Q_from, Config &Q_to)
{
  const auto K = Q_from[i]->neighbor.size();

  // exploit scatter data
  Vertex *prioritized_vertex = nullptr;
  if (scatter != nullptr) {
    auto itr_s = scatter->scatter_data[i].find(Q_from[i]->id);
    if (itr_s != scatter->scatter_data[i].end()) {
      prioritized_vertex = itr_s->second;
    }
  }

  // set C_next
  for (size_t k = 0; k < K; ++k) {
    auto u = Q_from[i]->neighbor[k];
    C_next[i][k] = u;
    tie_breakers[u->id] = get_random_float(MT);  // set tie-breaker
  }
  C_next[i][K] = Q_from[i];

  // sort, note: K + 1 is sufficient
  std::sort(C_next[i].begin(), C_next[i].begin() + K + 1,
            [&](Vertex *const v, Vertex *const u) {
              if (v == prioritized_vertex) return true;
              if (u == prioritized_vertex) return false;
              return D->get(i, v) + tie_breakers[v->id] <
                     D->get(i, u) + tie_breakers[u->id];
            });

  /* TODO: Add support for swap with field of view
  // emulate swap
  auto swap_agent = NO_AGENT;
  if (flg_swap) {
    swap_agent = is_swap_required_and_possible(i, Q_from, Q_to);
    if (swap_agent != NO_AGENT) {
      // reverse vertex scoring
      std::reverse(C_next[i].begin(), C_next[i].begin() + K + 1);
    }
  }

  auto swap_operation = [&]() {
    if (swap_agent != NO_AGENT &&                 // swap_agent exists
        Q_to[swap_agent] == nullptr &&            // not decided
        occupied_next[Q_from[i]->id] == NO_AGENT  // free
    ) {
      // pull swap_agent
      occupied_next[Q_from[i]->id] = swap_agent;
      Q_to[swap_agent] = Q_from[i];
    }
  };
  */

  // main loop
  for (size_t k = 0; k < K + 1; ++k) {
    std::set<size_t> affected_agents;
    bool fallback = false;
    auto u = C_next[i][k];

    // avoid vertex conflicts --- not really necessary, since it is included in
    // the can_occupy function, keeping it for readability.
    if (occupied_next[u->id] != NO_AGENT) continue;

    const auto j = occupied_now[u->id];

    // avoid swap conflicts with constraints --- needed only when field of view
    // is 0.
    if (j != NO_AGENT && Q_to[j] == Q_from[i]) continue;

    // avoid field of view conflicts
    if (!can_occupy(occupied_next, occupied_field_of_view_next, u, i)) continue;

    // reserve next location
    occupy(occupied_next, occupied_field_of_view_next, u, i);
    Q_to[i] = u;

    // priority inheritance

    // Move all agents that are in the field of view of the agent in their
    // current state. This includes the agent (if exists) that is currently at
    // u. (This can happen only when field of view is 0).
    std::set<size_t> agents_viewing_u_now =
        occupied_field_of_view_now[u->id].get_agents_occupied();
    for (size_t k : agents_viewing_u_now) {
      if (i == k) {
        continue;
      }

      if (nullptr == Q_to[k]) {
        std::set<size_t> affected_agents_k = funcPIBT(k, Q_from, Q_to);
        if (affected_agents_k.empty()) {
          fallback = true;
          // remove occupation for the next iterations:
          deoccupy(occupied_next, occupied_field_of_view_next, u, i);
          Q_to[i] = nullptr;
          break;  // need to fallback
        } else {
          // Union the sets:
          affected_agents.insert(affected_agents_k.begin(),
                                 affected_agents_k.end());
        }
      }
    }
    if (fallback) {
      for (size_t k : affected_agents) {
        deoccupy(occupied_next, occupied_field_of_view_next, Q_to[k], k);
        Q_to[k] = nullptr;
      }
      continue;
    }
    // success to plan next one step
    affected_agents.insert(i);
    return affected_agents;
  }
  // Failed to plan the next location for agent a_i, return empty set
  return std::set<size_t>();
}

std::set<size_t> PIBT::_set_next_vertex(const int i, const Config &Q_from,
                                        Config &Q_to, Vertex *u)
{
}

int PIBT::is_swap_required_and_possible(const int i, const Config &Q_from,
                                        Config &Q_to)
{
  // agent-j occupying the desired vertex for agent-i
  const auto j = occupied_now[C_next[i][0]->id];
  if (j != NO_AGENT && j != i &&  // j exists
      Q_to[j] == nullptr &&       // j does not decide next location
      is_swap_required(i, j, Q_from[i], Q_from[j]) &&  // swap required
      is_swap_possible(Q_from[j], Q_from[i])           // swap possible
  ) {
    return j;
  }

  // for clear operation, c.f., push & swap
  if (C_next[i][0] != Q_from[i]) {
    for (auto u : Q_from[i]->neighbor) {
      const auto k = occupied_now[u->id];
      if (k != NO_AGENT &&              // k exists
          C_next[i][0] != Q_from[k] &&  // this is for clear operation
          is_swap_required(k, i, Q_from[i],
                           C_next[i][0]) &&  // emulating from one step ahead
          is_swap_possible(C_next[i][0], Q_from[i])) {
        return k;
      }
    }
  }
  return NO_AGENT;
}

bool PIBT::is_swap_required(const int pusher, const int puller,
                            Vertex *v_pusher_origin, Vertex *v_puller_origin)
{
  auto v_pusher = v_pusher_origin;
  auto v_puller = v_puller_origin;
  Vertex *tmp = nullptr;
  while (D->get(pusher, v_puller) < D->get(pusher, v_pusher)) {
    auto n = v_puller->neighbor.size();
    // remove agents who need not to move
    for (auto u : v_puller->neighbor) {
      const auto i = occupied_now[u->id];
      if (u == v_pusher ||
          (u->neighbor.size() == 1 && i != NO_AGENT && ins->goals[i] == u)) {
        --n;
      } else {
        tmp = u;
      }
    }
    if (n >= 2) return false;  // able to swap at v_l
    if (n <= 0) break;
    v_pusher = v_puller;
    v_puller = tmp;
  }

  return (D->get(puller, v_pusher) < D->get(puller, v_puller)) &&
         (D->get(pusher, v_pusher) == 0 ||
          D->get(pusher, v_puller) < D->get(pusher, v_pusher));
}

bool PIBT::is_swap_possible(Vertex *v_pusher_origin, Vertex *v_puller_origin)
{
  // simulate pull
  auto v_pusher = v_pusher_origin;
  auto v_puller = v_puller_origin;
  Vertex *tmp = nullptr;
  while (v_puller != v_pusher_origin) {  // avoid loop
    auto n = v_puller->neighbor.size();
    for (auto u : v_puller->neighbor) {
      const auto i = occupied_now[u->id];
      if (u == v_pusher ||
          (u->neighbor.size() == 1 && i != NO_AGENT && ins->goals[i] == u)) {
        --n;
      } else {
        tmp = u;
      }
    }
    if (n >= 2) return true;  // able to swap at v_next
    if (n <= 0) return false;
    v_pusher = v_puller;
    v_puller = tmp;
  }
  return false;
}
