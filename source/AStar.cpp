#include "AStar.hpp"
#include <algorithm>
#include <queue>
#include <math.h>

using namespace std::placeholders;

namespace AStar {
    bool Vec2i::operator == (const Vec2i& coordinates_)
    {
        return (x == coordinates_.x && y == coordinates_.y);
    }

    Vec2i operator + (const Vec2i& left_, const Vec2i& right_)
    {
        return{ left_.x + right_.x, left_.y + right_.y };
    }

    Node::Node(Vec2i coordinates_, Node* parent_)
    {
        parent = parent_;
        coordinates = coordinates_;
        G = H = 0;
    }

    uint Node::getScore()
    {
        return G + H;
    }

    Generator::Generator()
    {
        setDiagonalMovement(false);
        setHeuristic(&Heuristic::manhattan);
        direction = {
            { 0, 1 }, { 1, 0 }, { 0, -1 }, { -1, 0 },
            { -1, -1 }, { 1, 1 }, { -1, 1 }, { 1, -1 }
        };
    }

    void Generator::setDiagonalMovement(bool enable_)
    {
        directions = (enable_ ? 8 : 4);
    }

    void Generator::setHeuristic(HeuristicFunction heuristic_)
    {
        heuristic = heuristic_;
    }

   
        // 尽量走直�?
        int calcNodeExtraCost(Node* currNode, const Point2i& nextNode, const Point2i& target) {
            // �?一�?点或直线�?
            if (currNode->parent == nullptr || nextNode.x == currNode->parent->coordinates.x
                || nextNode.y == currNode->parent->coordinates.y) {
                return 0;
            }

            // 拐向终点的点
            if (nextNode.x == target.x || nextNode.y == target.y) {
                return 1;
            }

            // �?通拐�?
            return 2;
        }

    uint Generator::getWeights(const Point2i& pos) const {
        auto it = weights_.find(pos);
        return it == weights_.end() ? DEFAULT_WEIGHT : it->second;
    }
    void Generator::setWeights(const Point2i& pos, uint weight) {
        weights_[pos] = weight;
    }


    CoordinateList Generator::findPath(Vec2i source_, Vec2i target_)
    {
        Node* current = nullptr;

        auto nodeComp = [](Node* a, Node* b) {
            return a->getScore() > b->getScore();
            };
        // 定义优先队列，按f值从小到大排�?; 小顶�?
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
                Vec2i newCoordinates(current->coordinates + direction[i]);
                if (map_.detectCollision(newCoordinates) ||
                    findNodeOnList(closedSet, newCoordinates) != nullptr) {
                    continue;
                }

                uint totalCost = current->G + ((i < 4) ? 10 : 14) * getWeights(newCoordinates)
                    + (directPrefer ? 10 * calcNodeExtraCost(current, newCoordinates, target_) : 0);

                Node* successor = findNodeOnList(openSet, newCoordinates);
                if (successor == nullptr) {
                    successor = new Node(newCoordinates, current);
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



    Node* Generator::findNodeOnList(NodeSet& nodes_, Vec2i coordinates_)
    {
        Node node(coordinates_);
        auto it = nodes_.find(&node);
        return it == nodes_.end() ? nullptr : *it;
    }

    void Generator::releaseNodes(NodeSet& nodes_)
    {
        for (auto it = nodes_.begin(); it != nodes_.end();) {
            delete* it;
            it = nodes_.erase(it);
        }
    }

    bool Map::detectCollision(Vec2i coordinates_)
    {
        // 有待优化
        if (coordinates_.x < 0 || coordinates_.x >= worldSize.x ||
            coordinates_.y < 0 || coordinates_.y >= worldSize.y ||
            walls.find(coordinates_) != walls.end()) {
            return true;
        }
        return false;
    }

    Vec2i Heuristic::getDelta(Vec2i source_, Vec2i target_)
    {
        return{ abs(source_.x - target_.x),  abs(source_.y - target_.y) };
    }

    uint Heuristic::manhattan(Vec2i source_, Vec2i target_)
    {
        auto delta = std::move(getDelta(source_, target_));
        return static_cast<uint>(10 * (delta.x + delta.y));
    }

    uint Heuristic::euclidean(Vec2i source_, Vec2i target_)
    {
        auto delta = std::move(getDelta(source_, target_));
        return static_cast<uint>(10 * sqrt(pow(delta.x, 2) + pow(delta.y, 2)));
    }

    uint Heuristic::octagonal(Vec2i source_, Vec2i target_)
    {
        auto delta = std::move(getDelta(source_, target_));
        return 10 * (delta.x + delta.y) + (-6) * std::min(delta.x, delta.y);
    }
}