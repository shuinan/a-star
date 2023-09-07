#ifndef __ROUTER_HPP_8F637DB91972F6C878D41D63F7E7214F__
#define __ROUTER_HPP_8F637DB91972F6C878D41D63F7E7214F__

#include <vector>
#include <set>
#include <queue>
#include <math.h>
#include <map>
#include <unordered_map>
#include <functional>
#include <algorithm>

#include <iostream>
#include <assert.h>

/// @brief 基础的jps算法需要走斜线
/// 考虑到我们有管廊/管网这一类捷径，可以贪心的方式直接走下去
/// 如果有空切的地方，直线扫描方式来定位, 朝向终点，前后上下直线搜索
/// ? 并找到边界点作为跳点
namespace Router
{
    using uint = unsigned int;

    struct Vec2i
    {
        int x, y;

        Vec2i() : x(0), y(0) {}
        Vec2i(int x0, int y0) : x(x0), y(y0) {}

        Vec2i operator + (const Vec2i& right_) const { return{ x + right_.x, y + right_.y }; }
        Vec2i operator - (const Vec2i& right_) const { return{ x - right_.x, y - right_.y }; }
        Vec2i& operator += (const Vec2i& right_) { x += right_.x; y += right_.y; return *this; }
        Vec2i& operator -= (const Vec2i& right_) { x -= right_.x; y -= right_.y; return *this; }
        bool operator == (const Vec2i& coordinates_) const { return x == coordinates_.x && y == coordinates_.y; }
        bool operator != (const Vec2i& coordinates_) const { return x != coordinates_.x || y != coordinates_.y; }
        bool operator < (const Vec2i& dest) const { return  x == dest.x ? y < dest.y : x < dest.x; }
        uint distance(const Vec2i& t) const { return (uint)sqrt(pow(x - t.x, 2) + pow(y - t.y, 2)); }
    };
    typedef Vec2i  Point2i;


    using HeuristicFunction = std::function<uint(Vec2i, Vec2i)>;
    using CoordinateList = std::vector<Vec2i>;

    struct Node
    {
        uint G, H;
        Vec2i coordinates;
        Node* parent;

        Node(Vec2i coord_, Node* parent_ = nullptr) {
            parent = parent_;
            coordinates = coord_;
            G = H = 0;
        }
        uint getScore() { return G + H; }
    };
    using NodeSet = std::set<Node*, std::function<bool(Node*, Node*)> >;

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

    ///  目前认为start < end的模式
    struct LineSeg {
        Point2i start;
        Point2i end;

        LineSeg& move(const Point2i offset) { start -= offset; end -= offset; return *this; }
        bool operator < (const LineSeg& dest) const { return (start.x == end.x) ? start.y < dest.start.y : start.x < dest.start.x; }
    };
    typedef LineSeg Bridge;
    /// @brief map
    class Map {
    public:
        static const uint BRIDGE_COST = 1;
        static const uint GENERAL_COST = 10;
        static const int MAX_WORLD_LEN = 100000;
    public:
        Map() : baseAlignPoint_({ 0,0 }) {}
        ~Map() { delete []wallsBuffer_; }

        bool baseCollision(const Point2i& nextCoord) {
            return  nextCoord.x < 0 || nextCoord.x >= worldSize_.x ||
                nextCoord.y < 0 || nextCoord.y >= worldSize_.y ||
                //walls_.find(nextCoord) != walls_.end();
                wallsBuffer_[nextCoord.x + nextCoord.y * worldSize_.x];
        }
        // 看看是否和以前规划的线路重复（通过path录入）
        bool detectCollision(const Point2i& nextCoord, const Point2i& curCoord) {
            if (baseCollision(nextCoord))
                return true;
            if (curCoord == nextCoord)
                return true;
            if (paths_.find(curCoord) != paths_.end() && paths_.find(nextCoord) != paths_.end())
                return false;

            return false;
        }
        // is there any collision between from and to. (include from, no to)
        bool isCollision(const Point2i& from, const Point2i& to) {
            int deltaX = std::abs(to.x - from.x);
            int deltaY = std::abs(to.y - from.y);
            int stepX = (to.x > from.x) ? 1 : -1;
            int stepY = (to.y > from.y) ? 1 : -1;
            int error = deltaX - deltaY;

            Point2i tempPt = from;
            while (tempPt != to) {
                if (baseCollision(tempPt))
                    return true;

                int error2 = 2 * error;
                if (error2 > -deltaY) {
                    error -= deltaY;
                    tempPt.x += stepX;
                }
                if (error2 < deltaX) {
                    error += deltaX;
                    tempPt.y += stepY;
                }
            }
            return false;
        }
        // is there any collision between from and to. (not include from and to)
        bool isLineCollision(const Point2i& from, const Point2i& to) {
            Vec2i unitDir(to - from);
            unitDir.x /= std::abs(unitDir.x + unitDir.y);
            unitDir.y /= std::abs(unitDir.x + unitDir.y);
            assert(unitDir.x == 0 || unitDir.y == 0);

            Point2i cur = from;
            while ((cur += unitDir) != to) {
                if (baseCollision(cur)) {
                    return true;
                }
            }
            return false;
        }

        // 最近的跳点，还必须是直连无障碍; 
        bool findJumpPoint(const Point2i& current, const Vec2i& dir, Point2i& jp) {
            int dis = MAX_WORLD_LEN;
            for (auto jumpPoint : jumpPoints_) {
                Vec2i curDir = jumpPoint - current;
                if (dir.x == 0 || dir.y == 0) {
                    // 射线搜索
                    if ((curDir.x == 0 || curDir.y == 0) && (curDir.x * dir.x > 0 || curDir.y * dir.y > 0)) {
                        if (isCollision(current, jumpPoint))
                            continue;
                        if (std::abs(curDir.x) + std::abs(curDir.y) < dis) {
                            dis = std::abs(curDir.x) + std::abs(curDir.y);
                            jp = jumpPoint;
                        }
                    }
                }
                // 从4斜边方向搜索小区域
                else if (curDir.x * dir.x > 0 && curDir.y * dir.y > 0) {
                    // 只要这两点之间无障碍就可以
                    if (isCollision(current, jumpPoint))
                        continue;
                    if (std::abs(curDir.x) + std::abs(curDir.y) < dis) {
                        dis = std::abs(curDir.x) + std::abs(curDir.y);
                        jp = jumpPoint;
                    }
                }
            }

            return dis != MAX_WORLD_LEN;
        }

        // 只有跳点加入搜索； 跳点可以借助一个点和bridge连接; 找到直线可以联通的桥； 从4直线方向搜索
        bool findNearestBridge(const Point2i& current, const Vec2i& dir, Point2i& jp) {
            assert(dir.x == 0 || dir.y == 0);

            int dis = MAX_WORLD_LEN;
            for (const auto& bridge : bridges_) {
                if (dir.x != 0) {
                    if (bridge.second.start.x != bridge.second.end.x) {
                        continue;
                    }
                    if ((bridge.second.start.x - current.x) * dir.x < 0) {
                        continue;
                    }
                    // 中间不能有障碍点                    
                    if (isCollision(current, { bridge.second.start.x, current.y })) {
                        continue;
                    }
                    if (current.y >= bridge.second.start.y && current.y <= bridge.second.end.y) {
                        if (dis > std::abs(bridge.second.start.x - current.x)) {
                            jp.x = bridge.second.start.x, jp.y = current.y;
                            dis = std::abs(jp.x - current.x);
                        }
                    }
                }
                else if (dir.y != 0) {
                    if (bridge.second.start.y != bridge.second.end.y) {
                        continue;
                    }
                    if ((bridge.second.start.y - current.y) * dir.y < 0) {
                        continue;
                    }
                    // 中间不能有障碍点                    
                    if (isCollision(current, { current.x, bridge.second.start.y })) {
                        continue;
                    }
                    if (current.x >= bridge.second.start.x && current.x <= bridge.second.end.x) {
                        if (dis > std::abs(bridge.second.start.y - current.y)) {
                            jp.x = current.x, jp.y = bridge.second.start.y;
                            dis = std::abs(jp.y - current.y);
                        }
                    }
                }
            }
            return dis != MAX_WORLD_LEN;
        }

        void preJumpPointSearch(const Point2i& target) {
            jumpPoints_.clear();

            // 四角斜对外点，认为是跳点（和jps里面可以走斜线的方式不同)
            Point2i jp;
            for (auto pt : walls_) {
                if (!baseCollision(pt + Vec2i(-1, 0)) && !baseCollision(pt + Vec2i(-1, -1)) && !baseCollision(pt + Vec2i(0, -1))) {
                    jp = { pt + Vec2i(-1, -1) };
                    jumpPoints_.insert(jp);
                }
                if (!baseCollision(pt + Vec2i(1, 0)) && !baseCollision(pt + Vec2i(1, -1)) && !baseCollision(pt + Vec2i(0, -1))) {
                    jp = { pt + Vec2i(1, -1) };
                    jumpPoints_.insert(jp);
                }
                if (!baseCollision(pt + Vec2i(0, 1)) && !baseCollision(pt + Vec2i(-1, 0)) && !baseCollision(pt + Vec2i(-1, 1))) {
                    jp = { pt + Vec2i(-1, 1) };
                    jumpPoints_.insert(jp);
                }
                if (!baseCollision(pt + Vec2i(1, 0)) && !baseCollision(pt + Vec2i(0, 1)) && !baseCollision(pt + Vec2i(1, 1))) {
                    jp = { pt + Vec2i(1, 1) };
                    jumpPoints_.insert(jp);
                }
                else {
                    continue;
                }
            }

            for (auto& dir : std::vector<Vec2i>({ { 1,0 },{ -1,0 },{ 0,1 },{ 0, -1} })) {
                if (findNearestBridge(target, dir, jp)) {
                    jumpPoints_.insert(jp);
                }
            }

            if (1) {
                for (const auto& bridge : bridges_) {
                    jumpPoints_.insert(bridge.second.start);
                    jumpPoints_.insert(bridge.second.start);
                }
            }
        }
        bool startFindPath(const Point2i& start, const Point2i& target) {
            if (!wallsBuffer_) {
                wallsBuffer_ = new(std::nothrow) char[worldSize_.x * worldSize_.y + 2];
                if (wallsBuffer_ == nullptr)
                    return false;
                memset(wallsBuffer_, 0, worldSize_.x * worldSize_.y);
                for (auto pt : walls_) {
                    assert(pt.x <= worldSize_.x && pt.y <= worldSize_.y);
                    assert(pt.x >= 0 && pt.y >= 0);
                    wallsBuffer_[pt.x + pt.y * worldSize_.x] = 1;
                }
            }
            preJumpPointSearch(target);
            jumpPoints_.insert(target);
            return true;
        }

        int addBridge(const Point2i& start, const Point2i& end) {
            // only vertical or horizen
            if (start.x == end.x) {
                int dir = start.y > end.y ? -1 : 1;
                for (int pos = 0; pos <= abs(start.y - end.y); pos++) {
                    costs_[{end.x, start.y + dir * pos}] = BRIDGE_COST;
                }
                bridges_[++bridgeId_] = start.y < end.y ? Bridge({ start, end }) : Bridge({ end, start });
            }
            else if (start.y == end.y) {
                int dir = start.x > end.x ? -1 : 1;
                for (int pos = 0; pos <= abs(start.x - end.x); pos++) {
                    costs_[{start.x + dir * pos, end.y}] = BRIDGE_COST;
                }
                bridges_[++bridgeId_] = start.x < end.x ? Bridge({ start, end }) : Bridge({ end, start });
            }

            return bridgeId_;
        }
        bool isBridge(const Point2i& pt) const { return getCost(pt) == BRIDGE_COST; }

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
            return it == costs_.end() ? GENERAL_COST : it->second;
        }
        void setCost(const Point2i& pos, uint cost) {
            costs_[pos] = cost;
        }

        void setWorldSize(Vec2i size) { worldSize_ = size; };
        void range(const Point2i& pt, Point2i& lbPoint, Point2i& rtPoint) {
            lbPoint.x = std::min(lbPoint.x, pt.x);
            rtPoint.x = std::max(rtPoint.x, pt.x);
            lbPoint.y = std::min(lbPoint.y, pt.y);
            rtPoint.y = std::max(rtPoint.y, pt.y);
        }
        /// @brief set worldsize by current walls, with extra w,h; will modify all position of current data
        /// 如果外部不能保证基准点是（0，0），应该先调用这个函数，再直线路径查找
        /// @param ew    extra width
        /// @param eh    extra height
        Vec2i align(int ew, int eh) {
            Point2i lbPoint({ 100000, 100000 }), rtPoint({ -100000, -100000 });
            for (auto& wall : walls_) {
                range(wall, lbPoint, rtPoint);
            }
            for (const auto& b : bridges_) {
                range(b.second.start, lbPoint, rtPoint);
                range(b.second.end, lbPoint, rtPoint);
            }
            lbPoint.x -= ew; rtPoint.x += ew;
            lbPoint.y -= eh; rtPoint.y += eh;

            worldSize_.x = rtPoint.x - lbPoint.x;
            worldSize_.y = rtPoint.y - lbPoint.y;

            auto tempWall = std::move(walls_);
            for (auto wall : tempWall) {
                walls_.insert(wall -= lbPoint);
            }
            auto tempPath = std::move(paths_);
            for (auto path : tempPath) {
                paths_.insert(path -= lbPoint);
            }
            for (auto& it : bridges_) {
                it.second.move(lbPoint);
            }

            auto temp = std::move(costs_);
            for (auto& it : temp) {
                costs_[it.first - lbPoint] = it.second;
            }

            baseAlignPoint_ = lbPoint;

            return lbPoint;
        }
        void addCollision(const Point2i& pt) { walls_.insert(pt); }
        /// @brief add a direct line
        void addCollisionLine(const Point2i& from, const Point2i& to) { addDirectLine(from, to, walls_); }
        void addCollisionRect(const Point2i& from, const Point2i& to) {
            addDirectLine(from, { to.x, from.y }, walls_);
            addDirectLine(from, { from.x, to.y }, walls_);
            addDirectLine(to, { to.x, from.y }, walls_);
            addDirectLine(to, { from.x, to.y }, walls_);
        }
        void removeCollision(const Point2i& coordinates) { walls_.erase(coordinates); }
        void clearCollisions() { walls_.clear(); }

        const Point2i& baseAlignPoint() const { return baseAlignPoint_; }

    private:
        template<class Container>
        void addDirectLine(const Point2i& from, const Point2i& to, Container& c) {
            // only vertical or horizen
            if (from.x == to.x) {
                int dir = from.y > to.y ? -1 : 1;
                for (int pos = 0; pos <= abs(from.y - to.y); pos++) {
                    c.insert({ to.x, from.y + dir * pos });
                }
            }
            else if (from.y == to.y) {
                int dir = from.x > to.x ? -1 : 1;
                for (int pos = 0; pos <= abs(from.x - to.x); pos++) {
                    c.insert({ from.x + dir * pos, to.y });
                }
            }
        }
    private:
        // 有待优化, 使用set节省空间，但是查找速度略慢      下面记录的都是点阵
        std::set<Point2i>       walls_;
        char*                   wallsBuffer_ = nullptr; // 使用这个buffer，速度比直接使用walls_速度提升一倍
        std::set<Point2i>       paths_;             // 如果规划多条线路，线路可以交叉，但是不可以有重复的段
        std::map<int, Bridge>   bridges_;
        int                     bridgeId_ = 0;
        Vec2i                   worldSize_;

        // 保存个点的权值，如果不在里面，就认为是缺省值：10；
        std::map<Point2i, uint> costs_;

        Point2i                 baseAlignPoint_;
        std::set<Point2i>       jumpPoints_;        // pre-searched jump points
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
            //directions = (enable_ ? 8 : 4);
        }

        Map& getMap() { return map_; }

        void setDirectPrefer(bool f) { directPrefer_ = f; }
        void setHeuristic(HeuristicFunction heuristic_) { heuristic = heuristic_; }

        bool isBridge(const Point2i& pt) { return map_.isBridge(pt); }

        /// @brief 目前的模式，有重复检测的问题，另外baseCollision效率太低了； 
        /// 后面考虑预处理的方式，把跳点先检测出来
        /// @param current 
        /// @param dir 
        /// @param jp       the jump point found
        /// @return 
        bool findJumpPoint_dynamic(Node* current, const Vec2i& dir, Point2i& jp) {
            jp = current->coordinates + dir;
            while (true)
            {
                if (map_.baseCollision(jp)) {
                    break;
                }

                /// if jump piont, 点或者矩形的四角斜方向的点，认为是跳点 
                Vec2i vertical({ dir.y, dir.x });
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

        uint findBridgeSegLen(const Point2i& start, const Point2i& end)
        {
            if (start == end)
                return 0;

            assert(end.x == start.x || end.y == start.y);

            Point2i bridgePt;
            Vec2i unitDir = end - start;
            unitDir.x /= std::abs(unitDir.x + unitDir.y);
            unitDir.y /= std::abs(unitDir.x + unitDir.y);

            bool findBridge = false;
            Point2i bridgeStart;
            Point2i bridgeEnd = start;
            while (!(bridgeEnd == end)) {
                if (map_.isBridge(bridgeEnd)) {
                    if (!findBridge) {
                        findBridge = true;
                        bridgeStart = bridgeEnd;
                    }
                }
                else if (findBridge) {
                    bridgeEnd = bridgeEnd - unitDir;
                    break;
                }

                bridgeEnd = bridgeEnd + unitDir;
            }

            return findBridge ? bridgeEnd.distance(bridgeStart) : 0;
        }
        bool midPointRouter(const Node* curNode, const Point2i& end, Point2i& midPoint, uint& extraCost, const Point2i& targetPt)
        {
            assert(curNode != nullptr);
            Point2i start = curNode->coordinates;
            Vec2i dir = end - start;
            Point2i midPointDx = start + Vec2i(dir.x, 0);
            uint bridgeLenDx = (findBridgeSegLen(start, midPointDx) + findBridgeSegLen(end, midPointDx)) * 10;

            Point2i midPointDy = start + Vec2i(0, dir.y);
            uint bridgeLenDy = (findBridgeSegLen(start, midPointDy) + findBridgeSegLen(end, midPointDy)) * 10;


            uint totalLen = Heuristic::manhattan(start, end);
            assert(bridgeLenDx <= totalLen && bridgeLenDy <= totalLen);

            // 优先走直线
            uint dxDist = totalLen * Map::GENERAL_COST + bridgeLenDx * Map::BRIDGE_COST - bridgeLenDx * Map::GENERAL_COST + (directPrefer_ ? calcNodeExtraCost(curNode, midPointDx, targetPt) : 0);
            uint dyDist = totalLen * Map::GENERAL_COST + bridgeLenDy * Map::BRIDGE_COST - bridgeLenDx * Map::GENERAL_COST + (directPrefer_ ? calcNodeExtraCost(curNode, midPointDy, targetPt) : 0);
            midPoint = dxDist > dyDist ? midPointDy : midPointDx;

            // 暂时认为cost是桥和普通模式
            extraCost = std::min(dxDist, dyDist);

            return dir.x != 0 && dir.y != 0;
        }

        /// <summary>
        /// 最有效率的查找方式： 通过找跳点，计算跳点到桥的最近点；   
        /// 如果没有数据调整，这个方法可以反复使用；  有地图数据加入或者删除时，主要是 align（对齐）会每次对现有整个地图对齐
        /// </summary>
        /// <param name="source_">寻路的开始点</param>
        /// <param name="target_">寻路的结束点</param>
        /// <returns>路径点，从开始点起，包括了开始点和结束点； 如果是空的表示没找到路径</returns>
        CoordinateList findPath(Point2i sourcePt, Point2i targetPt) {
            sourcePt -= map_.baseAlignPoint();
            targetPt -= map_.baseAlignPoint();

            if (!map_.startFindPath(sourcePt, targetPt))
                return CoordinateList();

            auto nodeComp = [](Node* a, Node* b) { return a->getScore() < b->getScore(); };
            // 按f值从小到大排序
            //std::priority_queue<Node*, std::vector<Node*>, std::function<bool(Node*, Node*)>> openQueue(nodeComp);
            NodeSet openQueue(nodeComp);

            auto setComp = [](Node* a, Node* b) { return b->coordinates < a->coordinates; };
            NodeSet closedSet(setComp);
            Node* startNode = new(std::nothrow) Node(sourcePt);
            if (startNode == nullptr) {
                return CoordinateList();
            }
            openQueue.insert(startNode);//openQueue.push(startNode);

            auto addNextPoint = [&](uint i, const Point2i& next, Node* parent) -> Node* {
                assert(parent != nullptr);

                if (map_.detectCollision(next, parent->coordinates) ||
                    findNodeOnList(closedSet, next) != nullptr) {
                    return nullptr;
                }

                // 一次可能跳跃多格
                uint extraCost = 0;
                // 如果拐弯了，还是需要把中间点也登记上         
                Point2i midPoint;
                if (midPointRouter(parent, next, midPoint, extraCost, targetPt)) {
                    Node* midNode = new(std::nothrow) Node(midPoint, parent);
                    if (midNode == nullptr)
                        return nullptr;
                    midNode->G = parent->G;
                    midNode->H = heuristic(midNode->coordinates, targetPt) * Map::GENERAL_COST / 2;
                    parent = midNode;
                }

                uint totalCost = parent->G + ((i < 4) ? 10 : 14) * extraCost / 10
                    + (directPrefer_ ? 10 * calcNodeExtraCost(parent, next, targetPt) : 0);

                Node* successor = nullptr;
                for (auto n : openQueue) {
                    if (n->coordinates == next) {
                        successor = n;
                        break;
                    }
                }
                if (successor == nullptr) {
                    successor = new(std::nothrow) Node(next, parent);
                    if (successor ==nullptr)
                        return nullptr;
                    successor->G = totalCost;
                    successor->H = heuristic(successor->coordinates, targetPt) * Map::GENERAL_COST /2;
                    openQueue.insert(successor);//openQueue.push(successor);
                }
                else if (totalCost < successor->G) {
                    openQueue.erase(successor);               // update order
                    successor->parent = parent;
                    successor->G = totalCost;                    
                    openQueue.insert(successor);
                }
                return successor;
            };

            Point2i jp;

            if (first4bridge_) {
                Node* nearestNode = nullptr;
                // 第一步优先最近的桥
                uint disBridge = Map::MAX_WORLD_LEN;
                for (uint i = 0; i < 4; ++i) {
                    if (map_.findNearestBridge(sourcePt, direction[i], jp)) {
                        Node* cur = addNextPoint(i, jp, startNode);
                        if (disBridge > jp.distance(sourcePt)) {
                            disBridge = jp.distance(sourcePt);
                            nearestNode = cur;
                        }
                    }
                }
                if (nearestNode != nullptr) {
                    openQueue.erase(nearestNode);    
                    nearestNode->G = 0;
                    openQueue.insert(nearestNode);    
                }
            }

            Node* current = nullptr;
            while (!openQueue.empty()) {
                current = *openQueue.begin();//current = openQueue.top();
                if (current->coordinates == targetPt) {
                    break;
                }

                closedSet.insert(current);
				openQueue.erase(current);//openQueue.pop();

                Point2i jp;                
                for (uint i = 0; i < directions; ++i) {
                    if (i < 4) {
                        if (isBridge(current->coordinates)) {
                            Vec2i nextCoord(current->coordinates + direction[i]);
                            // find next cross point
                            while (isBridge(nextCoord)) {
                                if (isBridge({ nextCoord.x + direction[i].y, nextCoord.y + direction[i].x }) ||
                                    isBridge({ nextCoord.x - direction[i].y, nextCoord.y - direction[i].x })) {
                                    addNextPoint(i, nextCoord, current);
                                    break;
                                }

                                nextCoord = nextCoord + direction[i];
                            }

                            if (map_.findJumpPoint(current->coordinates, direction[i], jp)) {
                                addNextPoint(i, jp, current);
                            }
                            continue;
                        }

                        if (map_.findNearestBridge(current->coordinates, direction[i], jp)) {
                            addNextPoint(i, jp, current);
                        }
                    }

                    // find jp(jump point)
                    bool haveJp = map_.findJumpPoint(current->coordinates, direction[i], jp);
                    if (haveJp) {
                        addNextPoint(i, jp, current);
                    }
                }
            }

            CoordinateList path, oriPath;
            if (current != nullptr && current->coordinates == targetPt) {
                while (current != nullptr) {
                    oriPath.push_back(current->coordinates);
                    current = current->parent;
                }
                std::reverse(oriPath.begin(), oriPath.end());

                if (oriPath.size() > 2) {
                    path.push_back(oriPath[0]);
                    std::size_t i = 2;
                    for (; i < oriPath.size(); ++i) {
                        if (oriPath[i].x != oriPath[i - 2].x && oriPath[i].y != oriPath[i - 2].y) {
                            path.push_back(oriPath[i - 1]);
                        }
                    }
                    path.push_back(oriPath.back());
                }
                for (auto& p : path) {
                    p += map_.baseAlignPoint();
                }
            }

			//while (!openQueue.empty()) {
            //    delete openQueue.top();
            //    openQueue.pop();
            //}
            for(auto n : openQueue)
                delete n;
            for (auto n : closedSet)
                delete n;

            return path;
        }

    private:
        Node* findNodeOnList(NodeSet& nodes_, Vec2i coordinates_) {
            Node node(coordinates_);
            auto it = nodes_.find(&node);
            return it == nodes_.end() ? nullptr : *it;
        }
        
        // 尽量走直线
        int calcNodeExtraCost(const Node* currNode, const Point2i& nextNode, const Point2i& target) {
            // 第一个点或直线点
            if (currNode->parent == nullptr || nextNode.x == currNode->parent->coordinates.x
                || nextNode.y == currNode->parent->coordinates.y) {
                return 0;
            }

            // 拐向终点的点
            if (nextNode.x == target.x || nextNode.y == target.y)
                return 1;

            // 普通拐点
            return 2;
        }

    private:
        Map                 map_;
        HeuristicFunction   heuristic;
        CoordinateList      direction;      // 所有方向（8个）
        uint                directions = 8; // 设置好可以走的方向数
        bool directPrefer_ = true;          // 是否优先走直线        
        bool first4bridge_ = true;          // 第一步是否优先走bridge
    };
}

#endif // __ROUTER_HPP_8F637DB91972F6C878D41D63F7E7214F__
