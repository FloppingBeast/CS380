#pragma once
#include "Misc/PathfindingDetails.hpp"

#define WIDTH 40
#define HEIGHT 40
#define PRINT false // Set to true to enable debug printing

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
  enum class List : uint8_t 
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

  // New path request
  void NewPathRequest(PathRequest& request);

  // Rubberbanding the path if requested
  void RubberbandPath(PathRequest& request);

  // Smoothing the path if requested
  void SmoothPath(PathRequest& request);

  // Function to call whenever the map changes to precompute any necessary data
  void OnMapChange();

  inline int index_for(const GridPos& pos) const {
    return pos.row * WIDTH + pos.col;
  }
  inline int index_for(int row, int col) const {
    return row * WIDTH + col;
  }
  inline GridPos pos_for(int index) {
    return { index / WIDTH, index % WIDTH };
  }

  // 40 by 40 grid representing the map
  Node grid[HEIGHT * WIDTH];

  std::vector<Node*> precomputedNeighbors[HEIGHT * WIDTH];

  Node* goalNode;

  class OpenList 
  {
  public:
    OpenList() : size(0) {}

    // increment last
    void push(Node* node) 
    {
      heap[size++] = node;
    }

    Node* pop() 
    {
      if (size > 0)
      {
        int cheapest = 0;

        for (int i = 1; i < size; ++i)
        {
          if (heap[i]->finalCost < heap[cheapest]->finalCost)
          {
            cheapest = i;
          }
        }

        // After finding cheapest index, replace with last element in list
        Node* returnNode = heap[cheapest];
        heap[cheapest] = heap[--size];

        if (PRINT)
        {
          std::cout << "Size: " << size << ", Popped Node: ("
            << returnNode->gridPos.row << ","
            << returnNode->gridPos.col << ") f="
            << returnNode->finalCost << "\n";
        }
        
        return returnNode;
      }

      return nullptr;
    }

    bool empty() const { return size == 0; }

    void clear() { size = 0; }

  private:
    Node* heap[HEIGHT * WIDTH * 2];
    int size;
  };

  OpenList openList;
};