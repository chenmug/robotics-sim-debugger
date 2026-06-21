#include "benchmarks/BenchmarkReporter.hpp"
#include <iostream>


// /*************** PRINT FINAL SUMMARY ***************/

void printFinalSummary(const std::vector<BenchmarkResult>& results)
{
    if (results.empty())
    {
        return;
    }

    double totalBfsReduction = 0.0;
    double totalDijkstraReduction = 0.0;

    for (const auto& r : results)
    {
        totalBfsReduction += (r.bfs.nodes - r.astar.nodes) / r.bfs.nodes * 100.0;

        totalDijkstraReduction += (r.dijkstra.nodes - r.astar.nodes) / r.dijkstra.nodes * 100.0;
    }

    double avgBfs = totalBfsReduction / results.size();
    double avgDijkstra = totalDijkstraReduction / results.size();
    double avgReduction = (avgBfs + avgDijkstra) / 2.0;

    std::cout << "\n\n===== FINAL SUMMARY =====\n\n";
    std::cout << "A* vs BFS/Dijkstra: ~"<< avgReduction << "% fewer nodes\n";
}