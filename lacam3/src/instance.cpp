#include "../include/instance.hpp"

Instance::~Instance()
{
    if (delete_graph_after_used) delete G;
}

Instance::Instance(Graph *_G, const Config &_starts, const Config &_goals,
                   uint _N, int _field_of_view_radius, size_t _k)
    : G(_G),
      starts(_starts),
      goals(_goals),
      N(_N),
      field_of_view_radius(_field_of_view_radius),
      k(_k)
{
}

Instance::Instance(const std::string &map_filename,
                   const std::vector<int> &start_indexes,
                   const std::vector<int> &goal_indexes,
                   int _field_of_view_radius, size_t _k)
    : G(new Graph(map_filename)),
      starts(Config()),
      goals(Config()),
      N(start_indexes.size()),
      field_of_view_radius(_field_of_view_radius),
      k(_k),
      delete_graph_after_used(true)
{
    for (auto k : start_indexes) starts.push_back(G->U[k]);
    for (auto k : goal_indexes) goals.push_back(G->U[k]);
}

// for load instance
static const std::regex r_instance =
    std::regex(R"(\d+\t.+\.map\t\d+\t\d+\t(\d+)\t(\d+)\t(\d+)\t(\d+)\t.+)");

Instance::Instance(const std::string &scen_filename,
                   const std::string &map_filename, const int _N,
                   int _field_of_view_radius, size_t _k)
    : G(new Graph(map_filename)),
      starts(Config()),
      goals(Config()),
      N(_N),
      field_of_view_radius(_field_of_view_radius),
      k(_k),
      delete_graph_after_used(true)
{
    // load start-goal pairs
    std::ifstream file(scen_filename);
    if (!file) {
        info(0, 0, scen_filename, " is not found");
        return;
    }
    std::string line;
    std::smatch results;

    while (getline(file, line)) {
        // for CRLF coding
        if (*(line.end() - 1) == 0x0d) line.pop_back();

        if (std::regex_match(line, results, r_instance)) {
            auto x_s = std::stoi(results[1].str());
            auto y_s = std::stoi(results[2].str());
            auto x_g = std::stoi(results[3].str());
            auto y_g = std::stoi(results[4].str());
            if (x_s < 0 || G->width <= x_s || x_g < 0 || G->width <= x_g)
                continue;
            if (y_s < 0 || G->height <= y_s || y_g < 0 || G->height <= y_g)
                continue;
            auto s = G->U[G->width * y_s + x_s];
            auto g = G->U[G->width * y_g + x_g];
            if (s == nullptr || g == nullptr) continue;
            starts.push_back(s);
            goals.push_back(g);
        }

        if (starts.size() == N) break;
    }
}

Instance::Instance(const std::string &map_filename, const int _N,
                   const int seed, int _field_of_view_radius, size_t _k)
    : G(new Graph(map_filename)),
      starts(Config()),
      goals(Config()),
      N(_N),
      field_of_view_radius(_field_of_view_radius),
      k(_k),
      delete_graph_after_used(true)
{
    auto MT = std::mt19937(seed);
    // random assignment
    const auto K = G->size();

    // set starts
    auto s_indexes = std::vector<int>(K);
    std::iota(s_indexes.begin(), s_indexes.end(), 0);
    std::shuffle(s_indexes.begin(), s_indexes.end(), MT);
    int i = 0;
    while (true) {
        if (i >= K) return;
        starts.push_back(G->V[s_indexes[i]]);
        if (starts.size() == N) break;
        ++i;
    }

    // set goals
    auto g_indexes = std::vector<int>(K);
    std::iota(g_indexes.begin(), g_indexes.end(), 0);
    std::shuffle(g_indexes.begin(), g_indexes.end(), MT);
    int j = 0;
    while (true) {
        if (j >= K) return;
        goals.push_back(G->V[g_indexes[j]]);
        if (goals.size() == N) break;
        ++j;
    }
}

bool Instance::is_valid(const int verbose) const
{
    if (N != starts.size() || N != goals.size()) {
        info(1, verbose, "invalid N, check instance");
        return false;
    }
    return true;
}

// Reads the solution from a .solution file in the format shown in mock.solution
void load_solution(const Instance &ins, Solution &solution,
                   const std::string &solution_file)
{
    std::ifstream file(solution_file);
    if (!file) {
        throw std::runtime_error("Cannot open solution file: " + solution_file);
    }

    std::string line;
    bool in_solution_section = false;
    std::regex timestep_regex(R"(^\d+:(.*),$)");
    std::regex coord_regex(R"(\((\d+),(\d+)\))");

    while (std::getline(file, line)) {
        if (!in_solution_section) {
            if (line == "solution=") {
                in_solution_section = true;
            }
            continue;
        }
        if (line.empty()) continue;
        std::smatch match;
        if (std::regex_match(line, match, timestep_regex)) {
            std::string agents_str = match[1];
            Config config;
            auto agents_begin = std::sregex_iterator(
                agents_str.begin(), agents_str.end(), coord_regex);
            auto agents_end = std::sregex_iterator();
            for (auto it = agents_begin; it != agents_end; ++it) {
                int x = std::stoi((*it)[1].str());
                int y = std::stoi((*it)[2].str());
                config.push_back(ins.G->U[ins.G->width * y + x]);
            }
            // Check if the configuration matches the number of agents
            if (config.size() != ins.N) {
                throw std::runtime_error(
                    "Configuration size does not match the number of agents: " +
                    std::to_string(config.size()) + " vs " +
                    std::to_string(ins.N));
            }
            solution.push_back(config);
        } else {
            // Stop if we reach a line that doesn't match the timestep format
            throw std::runtime_error("Invalid format in solution file: " +
                                     line);
        }
    }

    if (solution.empty()) {
        throw std::runtime_error(
            "No valid configurations found in the solution file.");
    }
}

Solution from_paths(const Paths paths)
{
    Solution solution;

    if (paths.empty()) {
        return solution;
    }

    // Determine the maximum path length (number of timesteps)
    size_t max_timesteps = 0;
    for (const auto &path : paths) {
        if (path.size() > max_timesteps) {
            max_timesteps = path.size();
        }
    }

    // Build the solution timestep by timestep
    for (size_t t = 0; t < max_timesteps; ++t) {
        Config config;
        config.reserve(paths.size());

        for (const auto &path : paths) {
            if (t < path.size()) {
                config.push_back(path[t]);
            } else {
                config.push_back(path.back());  // pad with goal vertex
            }
        }

        solution.push_back(std::move(config));
    }

    return solution;
}
