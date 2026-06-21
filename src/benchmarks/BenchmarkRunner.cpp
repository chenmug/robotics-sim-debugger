#include "benchmarks/BenchmarkRunner.hpp"
#include "benchmarks/RandomGridGenerator.hpp"
#include "core/SimulationEngine.hpp"
#include "robots/GridRobot.hpp"
#include "planners/BFSPlanner.hpp"
#include "planners/DijkstraPlanner.hpp"
#include "planners/AStarPlanner.hpp"
#include <memory>
#include <functional>


// /*************** HELPER FUNCTIONS ***************/

// /**************** RUN SINGLE *****************/

static AlgorithmStats runSingle(const std::vector<GridConfig>& dataset, std::function<std::shared_ptr<Planner>()> plannerFactory)
{
    double totalNodes = 0;
    double totalTime = 0;

    for (const auto& grid : dataset)
    {
        SimulationEngine engine(grid);

        auto planner = plannerFactory();
        auto robot = std::make_unique<GridRobot>(grid, planner);

        engine.addRobot(std::move(robot), {0, 0}, {grid.width - 1, grid.height - 1});

        while (!engine.allRobotsReached())
        {
            engine.runTick();
        }

        totalNodes += planner->getNodesExpanded();
        totalTime += planner->getLastRunTimeMs();
    }

    return {totalNodes / dataset.size(), totalTime / dataset.size()};
}


// /************* GENERATE DATASET **************/

static std::vector<GridConfig> generateDataset(const BenchmarkConfig& config)
{
    std::vector<GridConfig> dataset;

    for (int i = 0; i < config.runsPerConfig; ++i)
    {
        dataset.push_back(createRandomGrid(config.width, config.height, config.obstacleDensity));
    }

    return dataset;
}


// /*************** RUN BENCHMARK ***************/

BenchmarkResult runBenchmark(const BenchmarkConfig& config)
{
    BenchmarkResult result;

    result.label = "Grid:" + std::to_string(config.width) + "x" + std::to_string(config.height);

    result.obstacleInfo = std::to_string(int(config.obstacleDensity * 100)) + "% obstacles density";

    auto dataset = generateDataset(config);

    result.bfs = runSingle(dataset, [] {
        return std::make_shared<BFSPlanner>();
    });

    result.dijkstra = runSingle(dataset, [] {
        return std::make_shared<DijkstraPlanner>();
    });

    result.astar = runSingle(dataset, [] {
        return std::make_shared<AStarPlanner>();
    });

    return result;
}