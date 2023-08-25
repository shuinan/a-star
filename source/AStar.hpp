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

/// @brief 基础的jps算法需要走斜线
/// 考虑到我们有管廊/管网这一类捷径，可以贪心的方式直接走下去
/// 如果有空切的地方，直线扫描方式来定位, 朝向终点，前后上下直线搜索
/// ? 并找到边界点作为跳点
namespace AStar
{    
    struct Vec2i
    {
        int x, y;

        Vec2i() : x(0), y(0) {}
        Vec2i(int x0, int y0) : x(x0), y(y0) {}

        Vec2i operator + (const Vec2i& right_) const { return{ x + right_.x, y + right_.y }; }
        Vec2i operator - (const Vec2i& right_) const { return{ x - right_.x, y - right_.y }; }
        Vec2i& operator -= (const Vec2i& right_) { x -= right_.x; y -= right_.y; return *this; }
        bool operator == (const Vec2i& coordinates_) const { return (x == coordinates_.x && y == coordinates_.y); }
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
    
    /// @brief map
    class Map {
    public:
        typedef std::pair<Point2i, Point2i> Bridge;
        
    public:
        bool baseCollision(const Point2i& nextCoord) {            
            return  (nextCoord.x < 0 || nextCoord.x >= worldSize_.x ||
                     nextCoord.y < 0 || nextCoord.y >= worldSize_.y) ||
                     walls_.find(nextCoord) != walls_.end();
        }
        bool detectCollision(const Point2i& nextCoord, const Point2i& curCoord) {
            if (baseCollision(nextCoord))
                return true;
            if (walls_.find(nextCoord) != walls_.end())
                return true;
            if (curCoord == nextCoord)
                return true;
            if (paths_.find(curCoord) != paths_.end() && paths_.find(nextCoord) != paths_.end())
                return false;

            return false;
        }

        const int BridgeCost = 1;
        int addBridge(const Point2i& start, const Point2i& end) {
            // only vertical or horizen
            if (start.x == end.x) {
                int dir = start.y > end.y ? -1 : 1;
                for (int pos = 0; pos <= abs(start.y - end.y); pos++) {
                    costs_[{end.y, start.y + dir}] = BridgeCost;
                }
            } 
            else if (start.y == end.y) {
                int dir = start.x > end.x ? -1 : 1;
                for (int pos = 0; pos <= abs(start.x - end.x); pos++) {
                    costs_[{start.x+dir, end.y}] = BridgeCost;
                }
            }

            bridges_[++bridgeId_] = std::pair<Point2i, Point2i>(start, end);
            return bridgeId_;
        }
        
        /// @brief add a path which is composed by multi-segment
        /// @param path 
        void addPath(const std::vector<Point2i>& path) {
            if (path.size() < 2) {
                return;
            }
            for (uint32_t i = 1; i < path.size(); ++i) {
                addDirectLine(path[i - 1], path[i], paths_);
            }
        }

        uint getCost(const Point2i& pos) const {
            auto it = costs_.find(pos);
            return it == costs_.end() ? DEFAULT_COST : it->second;
        }
        void setCost(const Point2i& pos, uint cost) {
            costs_[pos] = cost;
        }

        void setWorldSize(Vec2i size) { worldSize_ = size; };
        /// @brief set worldsize by current walls, with extra w,h; will modify all position of current data
        /// @param ew    extra width
        /// @param eh    extra height
        Vec2i align(int ew, int eh) {
            Point2i lbPoint({100000, 100000}), rtPoint({-100000, -100000});
            for (auto& wall : walls_) {
                lbPoint.x = std::min(lbPoint.x, wall.x);
                rtPoint.x = std::max(rtPoint.x, wall.x);
                lbPoint.y = std::min(lbPoint.y, wall.y);
                rtPoint.y = std::max(rtPoint.y, wall.y);
            }
            lbPoint.x -= ew; rtPoint.x += ew;
            lbPoint.y -= eh; rtPoint.y += eh;

            worldSize_.x = rtPoint.x - lbPoint.x;
            worldSize_.y = rtPoint.y - lbPoint.y;

            auto tempWall = std::move(walls_);            
            for(auto wall : tempWall) {
                walls_.insert(wall -= lbPoint);
            }
            auto tempPath = std::move(paths_);
            for(auto path : tempPath) {
                paths_.insert(path -= lbPoint);
            }
            for(auto& it : bridges_) {
                it.second.first -= lbPoint;
                it.second.second -= lbPoint;
            }

            auto temp = std::move(costs_);
            for(auto& it : temp) {
                costs_[it.first - lbPoint] = it.second;
            }

            return lbPoint;
        }
        void addCollision(const Point2i& coordinates) { walls_.insert(coordinates); }
        /// @brief add a direct line
        void addCollisionLine(const Point2i& from, const Point2i& to) { addDirectLine(from, to, walls_); }
        void addCollisionRect(const Point2i& from, const Point2i& to) { 
            addDirectLine(from, {from.x, to.y}, walls_); 
            addDirectLine(from, {from.y, to.x}, walls_); 
            addDirectLine(to, {to.x, from.y}, walls_); 
            addDirectLine(to, {to.y, from.x}, walls_); 
        }
        void removeCollision(const Point2i& coordinates) { walls_.erase(coordinates); }
        void clearCollisions() { walls_.clear(); }

    private:
        template<class Container>
        void addDirectLine(const Point2i& from, const Point2i& to, Container& c) {
            // only vertical or horizen
            if (from.x == to.x) {
                int dir = from.y > to.y ? -1 : 1;
                for (int pos = 0; pos <= abs(from.y - to.y); pos++) {
                    c.insert({ to.y, from.y + dir });
                }
            }
            else if (from.y == to.y) {
                int dir = from.x > to.x ? -1 : 1;
                for (int pos = 0; pos <= abs(from.x - to.x); pos++) {
                    c.insert({ from.x + dir, to.y });
                }
            }
        }
    private:
        // 有待优化, 使用set节省空间，但是查找速度略慢      下面记录的都是点阵
        std::set<Point2i>   walls_;
        std::set<Point2i>   paths_;      // 如果规划多条线路，线路可以交叉，但是不可以有重复的段
        std::map<int, Bridge> bridges_;
        int bridgeId_ = 0;
        Vec2i               worldSize_;

        const uint DEFAULT_COST = 10;
        // 保存个点的权值，如果不在里面，就认为是缺省值：10；
        std::map<Point2i, uint> costs_;       
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

        bool isBridge(const Point2i& pt) { return map_.getCost(pt) == 1; }

        /// @brief 目前的模式，有重复检测的问题，另外baseCollision效率太低了； 
        /// 后面考虑预处理的方式，把跳点先检测出来
        /// @param current 
        /// @param dir 
        /// @param jp 
        /// @return 
        bool findJumpPoint(Node* current, const Vec2i& dir, Point2i& jp) {
            return false;
            jp = current->coordinates + dir;
            while (true)
            {                
                if (map_.baseCollision(jp)) {
                    break;
                }

                /// if jump piont, 点或者矩形的四角斜方向的点，认为是跳点 
                Vec2i vertical({dir.y, dir.x});
                if ((!map_.baseCollision(jp + dir) && !map_.baseCollision(jp + vertical) && map_.baseCollision(jp + dir + vertical)) ||
                    (!map_.baseCollision(jp + dir) && !map_.baseCollision(jp - vertical) && map_.baseCollision(jp + dir - vertical)) ||
                    (!map_.baseCollision(jp + vertical) && map_.baseCollision(jp - dir + vertical)) ||
                    (!map_.baseCollision(jp - vertical) && map_.baseCollision(jp - dir - vertical)) 
                )
                    return true; 

                jp = jp + dir;
            };
            
            return false;
        }
        
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

            auto addNextPoint = [&](uint i, const Point2i& next) ->bool {
                assert(current != nullptr);

                if (map_.detectCollision(next, current->coordinates) ||
                    findNodeOnList(closedSet, next) != nullptr) {
                    return false;
                }

                uint totalCost = current->G + ((i < 4) ? 10 : 14) * map_.getCost(next)
                    + (directPrefer ? 10 * calcNodeExtraCost(current, next, target_) : 0);

                Node* successor = findNodeOnList(openSet, next);
                if (successor == nullptr) {
                    successor = new Node(next, current);
                    successor->G = totalCost;
                    successor->H = heuristic(successor->coordinates, target_);
                    openQueue.push(successor);
                    openSet.insert(successor);
                }
                else if (totalCost < successor->G) {
                    successor->parent = current;
                    successor->G = totalCost;
                }
                return true;
            };

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
                    if (isBridge(current->coordinates)) {
                        while (isBridge(nextCoord)) {                            
                            // find end point or cross point
                            if (!isBridge(nextCoord) ||
                                isBridge({nextCoord.x + direction[i].y, nextCoord.y + direction[i].x}) ||
                                isBridge({nextCoord.x - direction[i].y, nextCoord.y - direction[i].x})) {
                                break;
                            }

                            nextCoord = nextCoord + direction[i];
                        }

                        addNextPoint(i, nextCoord);
                        continue;                             
                    }                    

                    // find jp(jump point)
                    Point2i jp;
                    bool haveJp = findJumpPoint(current, direction[i], jp);
                    if (haveJp) {
                        addNextPoint(i, jp);
                    } else {
                       addNextPoint(i, nextCoord);                       
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
