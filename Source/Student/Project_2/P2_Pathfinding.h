#pragma once
#include "Misc/PathfindingDetails.hpp"

#define WIDTH 40
#define HEIGHT 40

class AStarPather
{
public:
    /* 
        The class should be default constructible, so you might need to define a constructor.
        If needed, you can modify the framework where the class is constructed in the
        initialize functions of ProjectTwo and ProjectThree.
    */

    /* ************************************************** */
    // DO NOT MODIFY THESE SIGNATURES
    bool initialize();
    void shutdown();
    PathResult compute_path(PathRequest &request);
    /* ************************************************** */

    /*
        You should create whatever functions, variables, or classes you need.
        It doesn't all need to be in this header and cpp, structure it whatever way
        makes sense to you.
    */
private:
  
  // Enum for creation of unvisited, open, closed lists
  enum class List
  {
    eUnvisited = 0,
    eOpen,
    eClosed
  };

  struct Node
  {
    // Need to Connect to parent node
    Node* parent;

    GridPos gridPos;

    float givenCost;
    float finalCost;

    List onClosed;
  };

  // Helper Functions //

  // Used for clearing grid
  void ClearGrid( void );

  // Calculate heuristic
  float CalculateHeuristicCost(const Heuristic& heuristic, const Node& curr, const Node& end);

  // Calculate given cost
  float CalculateGivenCost(const Node* curr, const Node* next);

  // Find valid neighbors of a node
  void FindValidNeighbors(const Node* curr, std::vector<Node*>& neighbors);

  // 40 by 40 grid representing the map
  Node grid[HEIGHT][WIDTH];

  Node* goalNode;

  // Mock open list to get program running and working first
  class OpenList {
  public:
    void push(Node* node) { list.push_back(node); }
    Node* pop() {
      auto it = std::min_element(list.begin(), list.end(),
        [](Node* a, Node* b) { return a->finalCost < b->finalCost; });
      Node* n = *it;
      list.erase(it);
      return n;
    }
    void clear() { list.clear(); }
    bool empty() const { return list.empty(); }

  private:
    std::vector<Node*> list;
  };

  OpenList openList;
};