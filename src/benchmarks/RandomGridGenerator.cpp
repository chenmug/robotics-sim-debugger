#include "benchmarks/RandomGridGenerator.hpp"
#include <random>
#include <queue>
#include <unordered_set>


// /************** HELPER FUNCTION ***************/

static bool hasPath(const GridConfig& grid)
{
    std::queue<Position> q;
    std::unordered_set<int> visited;

    auto hash = [&grid](Position p)
    {
        return p.y * grid.width + p.x;
    };

    Position start{0, 0};
    Position goal{grid.width - 1, grid.height - 1};

    auto isBlocked = [&grid](Position p)
    {
        for (const auto& o : grid.static_obstacles)
        {
            if (o == p)
            { 
                return true;
            }
        }

        return false;
    };

    if (isBlocked(start) || isBlocked(goal))
    {
        return false;
    }

    q.push(start);
    visited.insert(hash(start));

    const int dx[4] = {1, -1, 0, 0};
    const int dy[4] = {0, 0, 1, -1};

    while (!q.empty())
    {
        auto cur = q.front();
        q.pop();

        if (cur == goal)
        {
            return true;
        }

        for (int i = 0; i < 4; ++i)
        {
            Position next{cur.x + dx[i], cur.y + dy[i]};

            if (next.x < 0 || next.y < 0 || next.x >= grid.width || next.y >= grid.height)
            {
                continue;
            }

            if (isBlocked(next))
            {
                continue;
            }

            int h = hash(next);
            if (visited.count(h))
            {
                continue;
            }

            visited.insert(h);
            q.push(next);
        }
    }

    return false;
}


// /************ CREATE RANDOM GRID **************/

GridConfig createRandomGrid(int width, int height, double obstacleDensity)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::bernoulli_distribution obstacle(obstacleDensity);

    GridConfig grid;
    grid.width = width;
    grid.height = height;

    do
    {
        grid.static_obstacles.clear();

        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                if ((x == 0 && y == 0) || (x == width - 1 && y == height - 1))
                {
                    continue;
                }

                if (obstacle(gen))
                {
                    grid.static_obstacles.push_back({x, y});
                }
            }
        }

    } while (!hasPath(grid));

    return grid;
}