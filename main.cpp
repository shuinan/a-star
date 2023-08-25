#include <iostream>
#include <chrono>
#include "source/AStar.hpp"

int main()
{    
     // 获取当前时间�?
    auto start = std::chrono::high_resolution_clock::now();

    AStar::Generator generator;
    generator.setHeuristic(AStar::Heuristic::euclidean);
    generator.setDiagonalMovement(false);
    
    AStar::Map& map = generator.getMap();
    map.setWorldSize({2500, 2500});
    map.addCollision({0, 4});
    map.addCollision({4, 0});
    map.addPath({{2,3}, {7,3}});

    for(int i=0; i<2400; ++i) {
        map.setCost({i, 2}, 1);
    }
    for(int i=0; i<2400; ++i) {
        map.setCost({2300, i}, 1);
    }

    std::cout << "Generate path ... \n";
    auto path = generator.findPath({0, 0}, {2400, 2400});

    for(auto& coordinate : path) {
        std::cout << coordinate.x << " " << coordinate.y << "\n";
    }


    // 获取当前时间�?
    auto end = std::chrono::high_resolution_clock::now();
    // 计算函数执�?�的时间
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    // 输出执�?�时间（以�??秒为单位�?
    std::cout << "time: " << duration.count() / 1000.0 << " ms" << std::endl;
}