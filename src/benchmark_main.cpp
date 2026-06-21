#include "benchmarks/BenchmarkSuite.hpp"
#include "benchmarks/BenchmarkRunner.hpp"
#include "benchmarks/BenchmarkReporter.hpp"
#include <iostream>


int main()
{
    std::cout << "\nRUNNING BENCHMARK EXECUTABLE\n";

    auto suite = createBenchmarkSuite();

    std::vector<BenchmarkResult> results;

    for (const auto& config : suite)
    {
        BenchmarkResult r = runBenchmark(config);
        results.push_back(r);

        std::cout << "\n============ " << r.label << " ============\n";
        std::cout << r.obstacleInfo << "\n\n";

        std::cout << "BFS:      " << r.bfs.nodes << " nodes | " << r.bfs.timeMs << " ms\n";
        std::cout << "Dijkstra: " << r.dijkstra.nodes << " nodes | " << r.dijkstra.timeMs << " ms\n";
        std::cout << "A*:       " << r.astar.nodes << " nodes | " << r.astar.timeMs << " ms\n";
    }

    printFinalSummary(results);

    return 0;
}