#include <iostream>
#include <chrono>
#include "source/AStar.hpp"

int main()
{
    auto start = std::chrono::high_resolution_clock::now();

    AStar::Generator generator;
    generator.setHeuristic(AStar::Heuristic::euclidean);
    generator.setDiagonalMovement(false);
    
    AStar::Map& map = generator.getMap();
    map.setWorldSize({2500, 2500});
    //map.addCollision({ 1399, 2 });
    map.addCollisionLine({4, 10}, {4, 100});
    map.addCollisionRect({ 1200, 1000 }, { 2200, 2200 });
    //map.addPath({{2,3}, {7,3}});
    map.addBridge({ 12, 32 }, { 2350,32 });
    map.addBridge({ 2300, 10 }, {2300, 2410});
    //map.setCost({100, 2}, 1);

    std::cout << "Generate path ... \n";
    auto path = generator.findPath({10, 2010}, {2400, 24});

    for(auto& coordinate : path) {
        std::cout << coordinate.x << " " << coordinate.y << "\n";
    }

#if 0
    path = generator.findPath({0, 0}, {1200, 1200});
    for(auto& coordinate : path) {
        std::cout << coordinate.x << " " << coordinate.y << "\n";
    }
#endif
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "time: " << duration.count() / 1000.0 << " ms" << std::endl;
}

