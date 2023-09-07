#include <iostream>
#include <chrono>
#include "source/Router.hpp"

void drawPath(const std::vector<std::pair<int, int>>& path, char* buffer, int width, int height) {
    // 将路径表示在缓冲区中
    for (std::size_t i = 0; i < path.size() - 1; i++) {
        int x1 = path[i].first;
        int y1 = path[i].second;
        int x2 = path[i + 1].first;
        int y2 = path[i + 1].second;

        if (x1 == x2) {
            // 绘制垂直线段
            for (int y = std::min(y1, y2); y <= std::max(y1, y2); y++) {
                buffer[y * width + x1] = '*';
            }
        }
        else if (y1 == y2) {
            // 绘制水平线段
            for (int x = std::min(x1, x2); x <= std::max(x1, x2); x++) {
                buffer[y1 * width + x] = '*';
            }
        }
    }

    // 打绘制好的缓冲区
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            printf("%c", buffer[y * width + x]);
        }
        printf("\n");
    }
}

int main()
{
    auto start = std::chrono::high_resolution_clock::now();

    Router::Generator generator;
    generator.setHeuristic(Router::Heuristic::euclidean);
    generator.setDiagonalMovement(false);
    
    Router::Map& map = generator.getMap();
    map.setWorldSize({260, 160});
    //map.addCollisionLine({4, 10}, {4, 100});
    map.addCollisionRect({ 30, 40 }, { 80, 90 });
    map.addCollisionRect({ 120, 50 }, { 160, 90 });
    map.addCollisionRect({ 210, 120 }, { 230, 140 });
    map.addCollisionRect({ 240, 110 }, { 255, 125 });
    map.addCollisionRect({ 110, 10 }, { 124, 23 });
    //map.addPath({{2,3}, {7,3}});
    map.addBridge({ 10, 25 }, { 254,25 });
    map.addBridge({ 100, 25 }, { 100,150 });
    map.addBridge({ 235, 20 }, {235, 158});

    std::cout << "Generate path ... \n";
    auto path = generator.findPath({20, 20}, {240, 130});

    for (int i = 1; i < 100; ++i)
        generator.findPath({ 20, 20 + i }, { 240, 130 });

    // 定义路径的点数组
    std::vector<std::pair<int, int>> path1;

    for(auto& coordinate : path) {
        std::cout << coordinate.x << " " << coordinate.y << "\n";
        path1.push_back({ coordinate.x/2, coordinate.y/2 });
    }

#if 0
    path = generator.findPath({0, 0}, {1200, 1200});
    for(auto& coordinate : path) {
        std::cout << coordinate.x << " " << coordinate.y << "\n";
    }
#endif
    // 定义地图的宽度和高度
    int width = 130;
    int height = 80;
    // 定义缓冲区
    char buffer[130 * 80];
    memset(buffer, '.', sizeof(buffer)); // 初始化缓冲区为 '.'
    // 将路径表示在缓冲区中并打印
    drawPath(path1, buffer, width, height);

    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "time: " << duration.count() / 1000.0 << " ms" << std::endl;
}

