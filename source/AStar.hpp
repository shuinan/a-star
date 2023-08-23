#ifndef __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__
#define __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__

#include <vector>
#include <set>
#include <queue>
#include <math.h>
#include <map>
#include <unordered_map>
#include <functional>
#include <algorithm>

#include <assert.h>

namespace AStar
{    
    struct Vec2i
    {
        int x, y;

        Vec2i operator + (const Vec2i& right_) const
        {
            return{ x + right_.x, y + right_.y };
        }
        bool operator == (const Vec2i& coordinates_) const {
            return (x == coordinates_.x && y == coordinates_.y);
        }
        bool operator < (const Vec2i& dest) const { return  x == dest.x ? y < dest.y : x < dest.x; }
    };
    typedef Vec2i  Point2i;

    
    using uint = unsigned int;
    using HeuristicFunction = std::function<uint(Vec2i, Vec2i)>;
    using CoordinateList = std::vector<Vec2i>;

    struct Node
    {
        uint G, H;
        Vec2i coordinates;
        Node *parent;

        Node(Vec2i coord_, Node* parent_ = nullptr) {
            parent = parent_;
            coordinates = coord_;
            G = H = 0;
        }
        uint getScore() { return G + H; }
    };

    using NodeSet = std::set<Node*, std::function<bool(Node*, Node*)>>;

     class Heuristic
    {
        static Vec2i getDelta(Vec2i source_, Vec2i target_) {
            return{ abs(source_.x - target_.x),  abs(source_.y - target_.y) };
        }

    public:
        static uint manhattan(Vec2i source_, Vec2i target_) {
            auto delta = std::move(getDelta(source_, target_));
            return static_cast<uint>(10 * (delta.x + delta.y));
        }
        // 欧几里得
        static uint euclidean(Vec2i source_, Vec2i target_) {
            auto delta = std::move(getDelta(source_, target_));
            return static_cast<uint>(10 * sqrt(pow(delta.x, 2) + pow(delta.y, 2)));
        }
        // 对角线
        static uint octagonal(Vec2i source_, Vec2i target_) {
            auto delta = std::move(getDelta(source_, target_));
            return 10 * (delta.x + delta.y) + (-6) * std::min(delta.x, delta.y);
        }
    };
    
    class Map {
    public:
        bool detectCollision(const Point2i& nextCoord, const Point2i& curCoord) {            
            if (nextCoord.x < 0 || nextCoord.x >= worldSize.x ||
                nextCoord.y < 0 || nextCoord.y >= worldSize.y) {
                if (walls.find(nextCoord) != walls.end()) 
                    return false;

                if (curCoord == nextCoord) 
                    return true;
                if (paths.find(curCoord) != paths.end() &&  paths.find(nextCoord) != paths.end())
                    return false;

                return true;
            }
            return false;
        }

        /// @brief add a path which is composed by multi-segment
        /// @param path 
        void addPath(const std::vector<Point2i>& path) {
            if (path.size() < 2) {
                return;
            }
            for(uint32_t i=1; i<path.size(); ++i) {
                // only vertical or horizen
                if (path[i-1].x == path[i].x) {
                    int dir = path[i-1].y > path[i].y ? -1 : 1;
                    for (int pos = 0; pos <= abs(path[i-1].y - path[i].y); pos++) {
                        paths.insert({path[i].y, path[i-1].y + dir});
                    }
                } 
                else if (path[i-1].y == path[i].y) {
                    int dir = path[i-1].x > path[i].x ? -1 : 1;
                    for (int pos = 0; pos <= abs(path[i-1].x - path[i].x); pos++) {
                        paths.insert({path[i-1].x+dir, path[i].y});
                    }
                }
            }
        }

        uint getWeights(const Point2i& pos) const {
            auto it = weights_.find(pos);
            return it == weights_.end() ? DEFAULT_WEIGHT : it->second;
        }
        void setWeights(const Point2i& pos, uint weight) {
            weights_[pos] = weight;
        }

        void setWorldSize(Vec2i worldSize_) { worldSize = worldSize_; };
        void addCollision(const Point2i& coordinates_) { walls.insert(coordinates_); }
        void removeCollision(const Point2i& coordinates_) { walls.erase(coordinates_); }
        void clearCollisions() { walls.clear(); }

    private:
        // 有待优化, 使用set节省空间，但是查找速度略慢      下面记录的都是点阵
        std::set<Point2i>   walls;
        std::set<Point2i>   paths;      // 如果规划多条线路，线路可以交叉，但是不可以有重复的段
        Vec2i               worldSize;

        const uint DEFAULT_WEIGHT = 10;
        // 保存个点的权值，如果不在里面，就认为是缺省值：10；
        std::map<Point2i, uint> weights_;
    };

    class Generator
    {  
    public:
        Generator() {
            setDiagonalMovement(false);
            setHeuristic(&Heuristic::manhattan);
            direction = {
                { 0, 1 }, { 1, 0 }, { 0, -1 }, { -1, 0 },
                { -1, -1 }, { 1, 1 }, { -1, 1 }, { 1, -1 }
            };
        }


        void setDiagonalMovement(bool enable_) {
            directions = (enable_ ? 8 : 4);
        }

        Map& getMap() { return map_; }

        void setDirectPrefer(bool f) { directPrefer = f; }
        void setHeuristic(HeuristicFunction heuristic_) { heuristic = heuristic_; }


        CoordinateList findPath(Vec2i source_, Vec2i target_) {
            Node* current = nullptr;

            auto nodeComp = [](Node* a, Node* b) {
                return a->getScore() > b->getScore();
                };
            // 定义优先队列，按f值从小到大排序; 小顶堆
            std::priority_queue<Node*, std::vector<Node*>, std::function<bool(Node*, Node*)>> openQueue(nodeComp);

            auto setComp = [](Node* a, Node* b) {
                return b->coordinates < a->coordinates;
                };
            NodeSet closedSet(setComp);
            NodeSet openSet(setComp);
            openQueue.push(new Node(source_));

            while (!openQueue.empty()) {
                current = openQueue.top();
                if (current->coordinates == target_) {
                    break;
                }

                closedSet.insert(current);
                openSet.erase(current);
                openQueue.pop();

                for (uint i = 0; i < directions; ++i) {
                    Vec2i nextCoord(current->coordinates + direction[i]);
                    if (map_.detectCollision(nextCoord, current->coordinates) ||
                        findNodeOnList(closedSet, nextCoord) != nullptr) {
                        continue;
                    }

                    uint totalCost = current->G + ((i < 4) ? 10 : 14) * map_.getWeights(nextCoord)
                        + (directPrefer ? 10 * calcNodeExtraCost(current, nextCoord, target_) : 0);

                    Node* successor = findNodeOnList(openSet, nextCoord);
                    if (successor == nullptr) {
                        successor = new Node(nextCoord, current);
                        successor->G = totalCost;
                        successor->H = heuristic(successor->coordinates, target_);
                        openQueue.push(successor);
                        openSet.insert(successor);
                    }
                    else if (totalCost < successor->G) {
                        successor->parent = current;
                        successor->G = totalCost;
                    }
                }
            }

            CoordinateList path;
            while (current != nullptr) {
                path.push_back(current->coordinates);
                current = current->parent;
            }

            while (!openQueue.empty())
            {
                delete openQueue.top();
                openQueue.pop();
            }
            releaseNodes(closedSet);

            return path;
        }      


    private:
        Node* findNodeOnList(NodeSet& nodes_, Vec2i coordinates_) {
            Node node(coordinates_);
            auto it = nodes_.find(&node);
            return it == nodes_.end() ? nullptr : *it;
        }
        void releaseNodes(NodeSet& nodes_) {
            for (auto it = nodes_.begin(); it != nodes_.end();) {
                delete* it;
                it = nodes_.erase(it);
            }
        }

        // 尽量走直线
        int calcNodeExtraCost(Node* currNode, const Point2i& nextNode, const Point2i& target) {
            // 第一个点或直线点
            if (currNode->parent == nullptr || nextNode.x == currNode->parent->coordinates.x
                || nextNode.y == currNode->parent->coordinates.y) {
                return 0;
            }

            // 拐向终点的点
            if (nextNode.x == target.x || nextNode.y == target.y) {
                return 1;
            }

            // 普通拐点
            return 2;
        }

    private:
        Map                 map_;
        HeuristicFunction   heuristic;
        CoordinateList      direction;      // 所有方向（8个）
        uint                directions;     // 设置好可以走的方向数
        bool directPrefer = true;           // 是否优先走直线        
    };   
}

#endif // __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__
