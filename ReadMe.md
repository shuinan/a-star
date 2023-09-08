## A* algorithm [![Build Status](https://travis-ci.org/daancode/a-star.svg?branch=master)](https://travis-ci.org/da-an/SHA-1)
A* search algorithm written in C++ programming language.
 - requires compiler support for C++11

#### Usage example
```cpp
#include <iostream>
#include "source/AStar.hpp"

int main()
{
    AStar::Generator generator;
    // Set 2d map size.
    generator.setWorldSize({25, 25});
    // You can use a few heuristics : manhattan, euclidean or octagonal.
    generator.setHeuristic(AStar::Heuristic::euclidean);
    generator.setDiagonalMovement(true);

    std::cout << "Generate path ... \n";
    // This method returns vector of coordinates from target to source.
    auto path = generator.findPath({0, 0}, {20, 20});

    for(auto& coordinate : path) {
        std::cout << coordinate.x << " " << coordinate.y << "\n";
    }
}
```
#### Preview
![](http://i.imgur.com/rqvrs6G.png)
![](http://i.imgur.com/7ZH2A0d.png)


使用 set 替代原来的数据结构，速度飙升。 目前debug模式下， 240/240 寻路，643ms

https://blog.csdn.net/wenyuan65/article/details/81587768  这篇文章有启发

不过，为提高速度，必须参考jps是思想

但是jps必须可以走斜线。
所以本项目自创了相关的算法。 目前效果良好。
