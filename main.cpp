#include <iostream>
#include <chrono>
#include "source/AStar.hpp"

int main()
{    
     // 获取当前时间点
    auto start = std::chrono::high_resolution_clock::now();

    AStar::Generator generator;
    generator.setWorldSize({250, 250});
    generator.setHeuristic(AStar::Heuristic::euclidean);
    generator.setDiagonalMovement(false);

    std::cout << "Generate path ... \n";
    auto path = generator.findPath({0, 0}, {240, 240});

    for(auto& coordinate : path) {
        std::cout << coordinate.x << " " << coordinate.y << "\n";
    }


    // 获取当前时间点
    auto end = std::chrono::high_resolution_clock::now();
    // 计算函数执行的时间
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    // 输出执行时间（以毫秒为单位）
    std::cout << "time: " << duration.count() / 1000.0 << " ms" << std::endl;
}