#include "benchmarks/BenchmarkSuite.hpp"


// /************ CREATE BENCHMARK SUITE **************/

std::vector<BenchmarkConfig> createBenchmarkSuite()
{
    return {
        {20, 20, 0.1, 50},
        {20, 20, 0.2, 50},
        {20, 20, 0.3, 50},
        {40, 40, 0.1, 50},
        {40, 40, 0.2, 50},
        {40, 40, 0.3, 50},
        {60, 60, 0.1, 50},
        {60, 60, 0.2, 50},
        {60, 60, 0.3, 50},
        {80, 80, 0.1, 50},
        {80, 80, 0.2, 50},
        {80, 80, 0.3, 50}
    };
}