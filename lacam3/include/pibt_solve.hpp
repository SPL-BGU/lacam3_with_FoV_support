#pragma once

#include "dist_table.hpp"
#include "graph.hpp"
#include "instance.hpp"
#include "pibt.hpp"
#include "post_processing.hpp"
#include "sipp.hpp"
#include "utils.hpp"

Solution pibt_solve(const Instance &ins, bool *solution_found,
                    const int verbose = 0, const Deadline *deadline = nullptr,
                    int seed = 0, int max_iter_count = 1000);

/**
 * @brief This struct is used to compare the order of agents in PIBT.
 *
 * @note The order is determined by first comparing the elapsed steps of the
 * agents, then by the initial distance to the goal, and finally by a tie
 * breaker value.
 *
 */
struct PIBTAgent {
    int elapsed;
    int init_distance;
    float tie_breaker;

    inline bool operator<(const PIBTAgent &other) const
    {
        if (elapsed != other.elapsed) {
            return elapsed < other.elapsed;
        }
        if (init_distance != other.init_distance) {
            return init_distance < other.init_distance;
        }
        return tie_breaker < other.tie_breaker;
    }
    inline bool operator>(const PIBTAgent &other) const
    {
        return other < *this;
    }
    inline bool operator<=(const PIBTAgent &other) const
    {
        return !(*this > other);
    }
    inline bool operator>=(const PIBTAgent &other) const
    {
        return !(*this < other);
    }
    inline bool operator==(const PIBTAgent &other) const
    {
        return (*this <= other) && (*this >= other);
    }
    inline bool operator!=(const PIBTAgent &other) const
    {
        return !(*this == other);
    }
};

/**
 * @brief Used to store PIBT agents in a vector.
 *
 */
using PIBTAgents = std::vector<PIBTAgent>;
