#pragma once
#include <memory>

struct GraphNode
{
  int x;
  int y;
  int cost;
  int heuristic;
  std::shared_ptr<GraphNode> prev;

  GraphNode() : GraphNode(0, 0) {};

  GraphNode(int in_x, int in_y) : x(in_x), y(in_y), cost(0) {}

  bool operator>(const GraphNode& other) const {
    return cost > other.cost;
  }

  bool operator==(const GraphNode& other) const {
    return x == other.x && y == other.y;
  }

  GraphNode operator+(std::pair <int, int> const & other) {
    GraphNode res(x + other.first, y + other.second);
    return res;
  }
};
