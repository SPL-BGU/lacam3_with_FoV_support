#include <execinfo.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <argparse/argparse.hpp>
#include <iostream>
#include <planner_choice.hpp>

void handler(int sig)
{
    void *array[20];
    size_t size;

    // get void*'s for all entries on the stack
    size = backtrace(array, 20);

    // print out all the frames to stderr
    fprintf(stderr, "Error: signal %d:\n", sig);
    backtrace_symbols_fd(array, size, STDERR_FILENO);
    exit(1);
}

int main(int argc, char *argv[])
{
    signal(SIGSEGV, handler);  // install our handler
    // arguments parser
    argparse::ArgumentParser program("lacam3", "0.1.0");
    program.add_argument("-p", "--planner")
        .help("planner type")
        .required()
        .choices("pibt", "lacam", "k_privacy_post_process")
        .nargs(1);
    program.add_argument("-m", "--map").help("map file").required();
    program.add_argument("-i", "--scen")
        .help("scenario file")
        .default_value(std::string(""));
    program.add_argument("-N", "--num").help("number of agents").required();
    program.add_argument("-s", "--seed")
        .help("seed")
        .default_value(std::string("0"));
    program.add_argument("-v", "--verbose")
        .help("verbose")
        .default_value(std::string("0"));
    program.add_argument("-t", "--time_limit_sec")
        .help("time limit sec")
        .default_value(std::string("3"));
    program.add_argument("-I", "--max_iter_count")
        .help("maximum number of iterations in PIBT")
        .default_value(std::string("10000"));
    program.add_argument("-o", "--output")
        .help("output file")
        .default_value(std::string("./build/result.txt"));
    program.add_argument("-l", "--log_short")
        .default_value(false)
        .implicit_value(true);
    program.add_argument("-r", "--field-of-view-radius")
        .help("Radius of field of view of each agent")
        .required();
    program.add_argument("-k", "--mock-agents-num")
        .help("Number of mock agents for each agent")
        .required();
    program.add_argument("-S", "--previous-solution")
        .help("previous solution file for k-privacy post-processing")
        .default_value(std::string(""));
    program.add_argument("-O", "--output-temporal-graph")
        .help("output temporal graph file")
        .default_value(std::string("./build/temporal_graph.map"));
    program.add_argument("-E", "--extended-safe-zones-output-name")
        .help("extended safe zones output file")
        .default_value(std::string("./build/result_extended_safe_zones.txt"));

    // solver parameters
    program.add_argument("--no-all")
        .help("turn off all options, i.e., vanilla LaCAM")
        .default_value(false)
        .implicit_value(true);
    program.add_argument("--no-star")
        .help("turn off the anytime part, i.e., usual LaCAM")
        .default_value(false)
        .implicit_value(true);
    program.add_argument("--random-insert-prob1")
        .help("probability of inserting the start node")
        .default_value(std::string("0.001"));
    program.add_argument("--random-insert-prob2")
        .help("probability of inserting a node after finding the goal")
        .default_value(std::string("0.01"));
    program.add_argument("--random-insert-init-node")
        .help("insert start node instead of random ones")
        .default_value(false)
        .implicit_value(true);
    program.add_argument("--no-swap")
        .help("turn off swap operation in PIBT")
        .default_value(false)
        .implicit_value(true);
    program.add_argument("--no-multi-thread")
        .help("turn off multi-threading")
        .default_value(false)
        .implicit_value(true);
    program.add_argument("--pibt-num")
        .help("used in Monte-Carlo configuration generation")
        .default_value(std::string("10"));
    program.add_argument("--no-scatter")
        .help("turn off SUO")
        .default_value(false)
        .implicit_value(true);
    program.add_argument("--scatter-margin")
        .help("allowing non-shortest paths in SUO")
        .default_value(std::string("10"));
    program.add_argument("--no-refiner")
        .help("turn off iterative refinement")
        .default_value(false)
        .implicit_value(true);
    program.add_argument("--refiner-num")
        .help("specify the number of refiners")
        .default_value(std::string("4"));
    program.add_argument("--recursive-rate")
        .help("specify the rate of the recursive call of LaCAM")
        .default_value(std::string("0.2"));
    program.add_argument("--recursive-time-limit")
        .help("time limit (sec) of the recursive call")
        .default_value(std::string("1"));
    program.add_argument("--checkpoints-duration")
        .help("for recording")
        .default_value(std::string("5"));

    program.add_argument("--ppfpp-threads")
        .help("number of threads for post-processing")
        .default_value(std::string("10"));
    try {
        program.parse_known_args(argc, argv);
    } catch (const std::runtime_error &err) {
        std::cerr << err.what() << std::endl;
        std::cerr << program;
        std::exit(1);
    }

    // setup instance
    const auto verbose = std::stoi(program.get<std::string>("verbose"));
    const auto time_limit_sec =
        std::stoi(program.get<std::string>("time_limit_sec"));
    const auto scen_name = program.get<std::string>("scen");
    const auto seed = std::stoi(program.get<std::string>("seed"));
    const auto map_name = program.get<std::string>("map");
    const auto output_name = program.get<std::string>("output");
    const auto log_short = program.get<bool>("log_short");
    const auto N = std::stoi(program.get<std::string>("num"));
    const auto field_of_view_radius =
        std::stoi(program.get<std::string>("field-of-view-radius"));
    if (field_of_view_radius < 0) {
        std::cerr << "field-of-view-radius must be positive or zero"
                  << std::endl;
        return 1;
    }
    const auto k = std::stoi(program.get<std::string>("mock-agents-num"));
    if (k <= 0) {
        std::cerr << "k must be positive" << std::endl;
        return 1;
    }
    const auto ins =
        scen_name.size() > 0
            ? Instance(scen_name, map_name, N, field_of_view_radius, k)
            : Instance(map_name, N, seed, field_of_view_radius, k);
    if (!ins.is_valid(1)) return 1;

    const auto planner_type =
        get_planner_type(program.get<std::string>("planner"));
    const auto max_iter_count =
        std::stoi(program.get<std::string>("max_iter_count"));
    if (max_iter_count <= 0) {
        std::cerr << "max_iter_count must be positive" << std::endl;
        return 1;
    }
    const auto previous_solution_file =
        program.get<std::string>("previous-solution");
    auto previous_solution = Solution();
    if (planner_type == PlannerType::KPrivacyPostProcess) {
        if (previous_solution_file.empty()) {
            std::cerr
                << "previous solution file is required for k-privacy post "
                   "processing"
                << std::endl;
            return 1;
        }
        // Load the previous solution from the file
        load_solution(ins, previous_solution, previous_solution_file);
    }
    const auto output_temporal_graph =
        program.get<std::string>("output-temporal-graph");
    const auto extended_safe_zones_output_filename =
        program.get<std::string>("extended-safe-zones-output-name");

    // solver parameters
    const auto flg_no_all = program.get<bool>("no-all");
    Planner::FLG_SWAP = !program.get<bool>("no-swap") && !flg_no_all;
    Planner::FLG_STAR = !program.get<bool>("no-star") && !flg_no_all;
    Planner::FLG_MULTI_THREAD =
        !program.get<bool>("no-multi-thread") && !flg_no_all;
    Planner::PIBT_NUM =
        flg_no_all ? 1 : std::stoi(program.get<std::string>("pibt-num"));
    Planner::FLG_REFINER = !program.get<bool>("no-refiner") && !flg_no_all;
    Planner::REFINER_NUM = std::stoi(program.get<std::string>("refiner-num"));
    Planner::FLG_SCATTER = !program.get<bool>("no-scatter") && !flg_no_all;
    Planner::SCATTER_MARGIN =
        std::stoi(program.get<std::string>("scatter-margin"));
    Planner::RANDOM_INSERT_PROB1 =
        flg_no_all ? 0
                   : std::stof(program.get<std::string>("random-insert-prob1"));
    Planner::RANDOM_INSERT_PROB2 =
        flg_no_all ? 0
                   : std::stof(program.get<std::string>("random-insert-prob2"));
    Planner::FLG_RANDOM_INSERT_INIT_NODE =
        program.get<bool>("random-insert-init-node") && !flg_no_all;
    Planner::RECURSIVE_RATE =
        flg_no_all ? 0 : std::stof(program.get<std::string>("recursive-rate"));
    Planner::RECURSIVE_TIME_LIMIT =
        flg_no_all
            ? 0
            : std::stof(program.get<std::string>("recursive-time-limit")) *
                  1000;
    Planner::CHECKPOINTS_DURATION =
        std::stof(program.get<std::string>("checkpoints-duration")) * 1000;

    KPrivacyPostProcess::FLG_ES_THREADS =
        flg_no_all ? 1 : std::stoi(program.get<std::string>("ppfpp-threads"));

    // solve
    if (planner_type == PlannerType::KPrivacyPostProcess) {
        KPrivacyPostProcess *kpp = nullptr;
        const auto deadline = Deadline(time_limit_sec * 1000);
        bool solution_found = false;
        std::tuple<Solution, Solution> post_process_solutions =
            solve_with_k_privacy_post_process(ins, &solution_found,
                                              previous_solution, verbose,
                                              &deadline, seed, &kpp);
        const auto comp_time_ms = deadline.elapsed_ms();
        // failure
        if (!solution_found) info(1, verbose, &deadline, "failed to solve");

        // check feasibility
        bool result = true;
        if (kpp == nullptr) {
            std::cerr
                << "KPrivacyPostProcess pointer is null, cannot proceed with "
                   "post-processing."
                << std::endl;
            result = false;
        }
        {
            std::ofstream temporal_graph_file(output_temporal_graph);
            kpp->print_safe_zones(temporal_graph_file);
        }
        if (!kpp->validate_k_privacy_post_process_solution(
                ins, std::get<0>(post_process_solutions), solution_found, true,
                verbose)) {
            info(0, verbose, &deadline,
                 "invalid IS safe zone solution after k-privacy "
                 "post-processing");
            result = false;
        }
        if (!kpp->validate_k_privacy_post_process_solution(
                ins, std::get<1>(post_process_solutions), solution_found, false,
                verbose)) {
            info(0, verbose, &deadline,
                 "invalid ES safe zone solution after k-privacy "
                 "post-processing");
            result = false;
        }
        delete kpp;     // Clean up the KPrivacyPostProcess object
        kpp = nullptr;  // Avoid dangling pointer
        if (!result) {
            return 1;  // Exit with error if the solution is not valid
        }
        // post processing
        // write log for IS solution:
        make_log(ins, std::get<0>(post_process_solutions), solution_found,
                 output_name, comp_time_ms, map_name, seed, log_short);
        // write log for ES solution:
        make_log(ins, std::get<1>(post_process_solutions), solution_found,
                 extended_safe_zones_output_filename, comp_time_ms, map_name,
                 seed, log_short);
    } else {
        const auto deadline = Deadline(time_limit_sec * 1000);
        bool solution_found = false;
        const auto solution =
            solve_with_planner(ins, planner_type, &solution_found, verbose - 1,
                               &deadline, seed, max_iter_count);
        const auto comp_time_ms = deadline.elapsed_ms();
        // failure
        if (!solution_found) info(1, verbose, &deadline, "failed to solve");

        // check feasibility
        if (!is_feasible_solution(ins, solution, solution_found, verbose)) {
            info(0, verbose, &deadline, "invalid solution");
            return 1;
        }

        // post processing
        print_stats(verbose, &deadline, ins, solution, comp_time_ms);
        make_log(ins, solution, solution_found, output_name, comp_time_ms,
                 map_name, seed, log_short);
    }

    return 0;
}
