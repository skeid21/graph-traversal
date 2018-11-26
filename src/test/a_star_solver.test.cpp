#include "../a_star_solver.h"

#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <list>

class GraphNode;
using TestAStarSovler = AStarSolver<GraphNode, double>;
using graph_node_ptr_t = std::shared_ptr<GraphNode>;

class GraphNode
{
  public:
    double xPos;
    double yPos;
    std::list<graph_node_ptr_t> neighbors;

    double calculateDistance(GraphNode *other)
    {
        auto xDiff = other->xPos - xPos;
        auto yDiff = other->yPos - yPos;
        return std::sqrt((xDiff * xDiff) + (yDiff * yDiff));
    }
    double calculateGCost(GraphNode *other)
    {
        return calculateDistance(other);
    }

    double calculateHCost(GraphNode *other)
    {
        return calculateDistance(other);
    }

    void visitNeighbors(TestAStarSovler::visit_node_function_t visitor)
    {
        for (auto neighbor : neighbors)
        {
            visitor(neighbor.get());
        }
    }
};

static const size_t GRID_SIZE = 50;
std::vector<std::vector<graph_node_ptr_t>> createGridOfGraphNodes()
{
    std::vector<std::vector<graph_node_ptr_t>> graphNodes;
    for(size_t yScan = 0; yScan < GRID_SIZE; ++yScan)
    {
        std::vector<graph_node_ptr_t> currentRow;
        for(size_t xScan = 0; xScan < GRID_SIZE; ++xScan)
        {
           auto node = std::make_shared<GraphNode>();
           node->xPos = xScan;
           node->yPos = yScan; 
           currentRow.push_back(node);
        }
        graphNodes.emplace_back(std::move(currentRow));
    }

    for(size_t yScan = 0; yScan < GRID_SIZE; ++yScan)
    {
        auto currentRow = graphNodes[yScan];
        for(size_t xScan = 0; xScan < GRID_SIZE; ++xScan)
        {
            auto currentNode = currentRow[xScan];
            if (xScan > 0) 
            {
                auto n =  currentRow[xScan -1];
                currentNode->neighbors.push_back(n);
            }

            if (xScan < GRID_SIZE-1 )
            {
                auto n = currentRow[xScan + 1];
                currentNode->neighbors.push_back(n);
            }

            if (yScan > 0) 
            {
                auto n = graphNodes[yScan -1][xScan];
                currentNode->neighbors.push_back(n);
            }

            if (yScan < GRID_SIZE - 1)
            {
                auto n = graphNodes[yScan + 1][xScan];
                currentNode->neighbors.push_back(n);
            }
        }
    }

    return graphNodes;
}
        

TEST(AStarSolver, StartNodeIsEndNode)
{
    auto node = std::make_shared<GraphNode>();
    auto sut = AStarSolver<GraphNode, double>();
    auto res = sut.solvePath(node.get(), node.get());

    ASSERT_FALSE(res.empty());
    ASSERT_EQ(*res.begin(), node.get());
}

TEST(AStarSolver, PathIsFoundForSingleListOfNodes)
{
    auto startNode = std::make_shared<GraphNode>();
    startNode->xPos = 0; 
    startNode->yPos = 0;
    graph_node_ptr_t currentNode = startNode;
    for(int scan = 1; scan < 10; ++scan) {
        auto newNode = std::make_shared<GraphNode>();
        newNode->xPos = scan;
        newNode->yPos = scan;
        currentNode->neighbors.push_back(newNode);
        currentNode = newNode;
    }

    auto endNode = currentNode;

    auto sut = AStarSolver<GraphNode, double>();
    auto res = sut.solvePath(startNode.get(), endNode.get());

    auto resVector = std::vector<GraphNode*>(res.begin(), res.end());
    ASSERT_EQ(resVector.size(), 10);
    ASSERT_EQ(resVector[0], startNode.get());
    ASSERT_EQ(resVector[9], endNode.get());
}

TEST(AStarSolver, PathIsFoundForSingleListOfNodesWithDeadEndNeighbors)
{
    auto startNode = std::make_shared<GraphNode>();
    startNode->xPos = 0; 
    startNode->yPos = 0;
    graph_node_ptr_t currentNode = startNode;
    for(size_t scan = 1; scan < 10; ++scan) {
        auto newNode = std::make_shared<GraphNode>();
        newNode->xPos = scan;
        newNode->yPos = scan;
        currentNode->neighbors.push_back(newNode);
        for(size_t deadEndNeighborsScan = 2; deadEndNeighborsScan < 7; ++deadEndNeighborsScan)
        {
            auto deadEndNode = std::make_shared<GraphNode>();
            deadEndNode->xPos = scan;
            deadEndNode->yPos = scan * deadEndNeighborsScan;
            currentNode->neighbors.push_back(deadEndNode);
        }
        currentNode = newNode;
    }

    auto endNode = currentNode;

    auto sut = AStarSolver<GraphNode, double>();
    auto res = sut.solvePath(startNode.get(), endNode.get());

    auto resVector = std::vector<GraphNode*>(res.begin(), res.end());
    ASSERT_EQ(resVector.size(), 10);
    ASSERT_EQ(resVector[0], startNode.get());
    ASSERT_EQ(resVector[9], endNode.get());
}

TEST(AStarSolver, NoPathIsFoundWithDisjointGraph)
{
    auto startNode = std::make_shared<GraphNode>();
    startNode->xPos = 0; 
    startNode->yPos = 0;
    
    auto endNode = std::make_shared<GraphNode>();
    endNode->xPos = 100; 
    endNode->yPos = 100;

    auto sut = AStarSolver<GraphNode, double>();
    auto res = sut.solvePath(startNode.get(), endNode.get());
    ASSERT_TRUE(res.empty());
}

TEST(AStarSolver, PathFoundCornerToCornerForGridOfNodes)
{
    auto graphNodes = createGridOfGraphNodes();
    auto startNode  = graphNodes[0][0];
    auto endNode = graphNodes[GRID_SIZE-1][GRID_SIZE-1];
    auto sut = AStarSolver<GraphNode, double>();
    auto res = sut.solvePath(startNode.get(), endNode.get());

    auto resVector = std::vector<GraphNode*>(res.begin(), res.end());
    
    ASSERT_FALSE(resVector.empty());
    ASSERT_EQ(resVector.size(), 99);
    ASSERT_EQ(resVector[0], startNode.get());
    ASSERT_EQ(resVector[resVector.size()-1], endNode.get());
}

