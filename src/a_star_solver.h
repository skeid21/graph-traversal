#pragma once
#include <unordered_map>
#include <queue>
#include <vector>
#include <forward_list>
#include <unordered_set>
#include <memory>
#include <iostream>

//Exploration of the A* search algorithm using std containers and templates 
template<typename NODE_TYPE, typename UNIT_TYPE> 
class AStarSolver
{
    class SearchNode;
    using search_node_ptr_t = std::shared_ptr<SearchNode>;
    class SearchNode 
    {
    public:
        NODE_TYPE* node;
        search_node_ptr_t fromNode;
        UNIT_TYPE fScore;
        UNIT_TYPE gScore;
    };

    struct FScoreLess
    {
        bool operator() (const search_node_ptr_t& lhv, const search_node_ptr_t& rhv) const
        {
            return lhv->fScore > rhv->fScore;
        }
    };
    
    std::forward_list<NODE_TYPE*> reconstructPath(const search_node_ptr_t endNode)
    {
        std::forward_list<NODE_TYPE*> path;
        search_node_ptr_t current = endNode;

        while(nullptr != current) {
            path.push_front(current->node);
            current = current->fromNode;
        }

        return path;
    }

public:
    using visit_node_function_t = std::function<void(NODE_TYPE*)>;
    std::forward_list<NODE_TYPE*> solvePath(NODE_TYPE* _start, NODE_TYPE* _end)
    {
        std::unordered_set<NODE_TYPE*> closedSet;
        std::priority_queue<search_node_ptr_t, std::vector<search_node_ptr_t>, FScoreLess> openSetPriorityQueue;
        std::unordered_map<NODE_TYPE*, search_node_ptr_t> openSet;

        auto startingNode = std::make_shared<SearchNode>();
        startingNode->node = _start;
        startingNode->gScore = 0.f;
        startingNode->fScore = _start->calculateHCost(_end);
        openSetPriorityQueue.push(startingNode);
        openSet.insert({startingNode->node, startingNode});

        std::forward_list<NODE_TYPE*> path;
        while(false == openSet.empty()) 
        {
            auto current = openSetPriorityQueue.top();
            if (current->node == _end) 
            {
                path = reconstructPath(current); 
                break;
            }

            openSetPriorityQueue.pop();
            openSet.erase(current->node);
            closedSet.insert(current->node);

            current->node->visitNeighbors([&](NODE_TYPE* neighbor){
                if (closedSet.find(neighbor) != closedSet.end()) {
                    return;
                }

                auto gScore = current->gScore + current->node->calculateGCost(neighbor);
                auto findItr = openSet.find(neighbor);
                search_node_ptr_t neighborSearchNode = nullptr;
                if (findItr == openSet.end()) 
                {
                    neighborSearchNode =std::make_shared<SearchNode>();
                    neighborSearchNode->node = neighbor;
                    openSet.insert({neighbor, neighborSearchNode});
                }
                else 
                {
                    neighborSearchNode = findItr->second;
                    if (gScore >= neighborSearchNode->gScore)
                    {
                        return;
                    }
                }

                neighborSearchNode->fromNode = current;
                neighborSearchNode->gScore = gScore;
                neighborSearchNode->fScore = gScore + neighbor->calculateHCost(_end);
                openSetPriorityQueue.push(neighborSearchNode);
            });
        }

        return path;
    }
};