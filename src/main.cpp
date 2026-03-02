#include "motion_planner/astar.hpp"
#include "motion_planner/grid.hpp"
#include "motion_planner/visualizer.hpp"
#include <iostream>

int main() {
    using namespace motion_planner;

    Grid grid(100, 100);

    grid.addWall(30, 0,  30, 70);
    grid.addWall(60, 30, 60, 99);
    grid.addWall(30, 70, 50, 70);

    Point2D start{5, 5};
    Point2D goal{95, 95};

    AStar planner(grid);
    PlanResult result = planner.plan(start, goal);

    if (result.success)
        std::cout << "Path found! Steps: " << result.path.size()
                  << " | Cost: " << result.cost << "\n";
    else
        std::cout << "No path found.\n";

    Visualizer visualizer(grid, 6);
    visualizer.show(result, start, goal, "A* Result");
    visualizer.save("astar_result.png", result, start, goal);

    return 0;
}