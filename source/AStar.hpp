/*
    Copyright (c) 2015, Damian Barczynski <daan.net@wp.eu>
    Following tool is licensed under the terms and conditions of the ISC license.
    For more information visit https://opensource.org/licenses/ISC.
*/
#ifndef __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__
#define __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__

#include <vector>
#include <functional>
#include <set>
#include <unordered_map>
#include <map>

namespace AStar
{    
    struct Vec2i
    {
        int x, y;

        bool operator == (const Vec2i& coordinates_);
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

        Node(Vec2i coord_, Node *parent_ = nullptr);
        uint getScore();
    };

    using NodeSet = std::set<Node*, std::function<bool(Node*, Node*)>>;
    
    struct classcomp {
        bool operator() (const int& lhs, const int& rhs) const
        {
            return lhs<rhs;
        }
    };


    class Generator
    {
        bool detectCollision(Vec2i coordinates_);
        Node* findNodeOnList(NodeSet& nodes_, Vec2i coordinates_);
        void releaseNodes(NodeSet& nodes_);

    public:
        Generator();
        void setWorldSize(Vec2i worldSize_);
        void setDiagonalMovement(bool enable_);
        void setHeuristic(HeuristicFunction heuristic_);
        CoordinateList findPath(Vec2i source_, Vec2i target_);
        void addCollision(Vec2i coordinates_);
        void removeCollision(Vec2i coordinates_);
        void clearCollisions();

        uint getWeights(const Point2i& pos) const;
        void setWeights(const Point2i& pos, uint weight);

        void setDirectPrefer(bool f) { directPrefer = f; }

    private:
        HeuristicFunction heuristic;
        CoordinateList direction, walls;
        Vec2i worldSize;
        uint directions;
        bool directPrefer = true;           // 是否优先走直线
        const uint DEFAULT_WEIGHT = 10;
        // 保存个点的权值，如果不在里面，就认为是缺省值：10；
        std::map<Point2i, uint> weights_;
    };

    class Heuristic
    {
        static Vec2i getDelta(Vec2i source_, Vec2i target_);

    public:
        static uint manhattan(Vec2i source_, Vec2i target_);
        // 欧几里得
        static uint euclidean(Vec2i source_, Vec2i target_);
        // 对角线
        static uint octagonal(Vec2i source_, Vec2i target_);
    };
}

#endif // __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__
