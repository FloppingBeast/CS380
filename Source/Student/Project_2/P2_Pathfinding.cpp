#include <pch.h>
#include "Projects/ProjectTwo.h"
#include "P2_Pathfinding.h"

#pragma region Extra Credit 
bool ProjectTwo::implemented_floyd_warshall()
{
    return false;
}

bool ProjectTwo::implemented_goal_bounding()
{
    return false;
}
#pragma endregion


bool AStarPather::initialize()
{
    // handle any one-time setup requirements you have

    /*
        If you want to do any map-preprocessing, you'll need to listen
        for the map change message.  It'll look something like this:

        Callback cb = std::bind(&AStarPather::your_function_name, this);
        Messenger::listen_for_message(Messages::MAP_CHANGE, cb);

        There are other alternatives to using std::bind, so feel free to mix it up.
        Callback is just a typedef for std::function<void(void)>, so any std::invoke'able
        object that std::function can wrap will suffice.
    */

    return true; // return false if any errors actually occur, to stop engine initialization
}

void AStarPather::shutdown()
{
    /*
        Free any dynamically allocated memory or any other general house-
        keeping you need to do during shutdown.
    */
}

PathResult AStarPather::compute_path(PathRequest& request)
{
  /*
  If (request.newRequest) {
      Initialize everything. Clear Open/Closed Lists.
      Push Start Node onto the Open List with cost of f(x) = g(x) + h(x).
  }
  While (Open List is not empty) {
    parentNode = Pop cheapest node off Open List.
    If parentNode is the Goal Node, then path found (return PathResult::COMPLETE).
    Place parentNode on the Closed List.
    For (all neighboring child nodes of parentNode) {
      Compute its cost, f(x) = g(x) + h(x)
      If child node isn’t on Open or Closed list, put it on Open List.
      Else if child node is on Open or Closed List, AND this new one is cheaper,
        then take the old expensive one off both lists and put this new
        cheaper one on the Open List.
      }
      If taken too much time this frame (or if request.settings.singleStep == true),
         abort search for now and resume next frame (return PathResult::PROCESSING).
  }
  Open List empty, thus no path possible (return PathResult::IMPOSSIBLE).

  */

  // Check to see if new path has been requested
  if (request.newRequest) 
  {
    // Clear the grid
    ClearGrid();

    // Clear the open list
    openList.clear();

    // Push Start Node onto the Open List 
    GridPos startPos = terrain->get_grid_position(request.start);
    Node& startNode = grid[startPos.row][startPos.col];
    startNode.gridPos = startPos;
    startNode.onClosed = List::eOpen;
    startNode.parent = nullptr;
    openList.push(&startNode);

    // Update the end node position
    GridPos endPos = terrain->get_grid_position(request.goal);
    Node& endNode = grid[endPos.row][endPos.col];
    endNode.gridPos = endPos;
    goalNode = &endNode;
    
    // Calculate final cost f(x) = g(x) + h(x).
    startNode.givenCost = 0.0f;
    startNode.finalCost = startNode.givenCost + CalculateHeuristicCost(request.settings.heuristic, startNode, endNode);
  }

  // While Open List is not empty
  //  parentNode = Pop cheapest node off Open List.
  //  If parentNode is the Goal Node, then path found(return PathResult::COMPLETE).
  //  Place parentNode on the Closed List.
  //  For(all neighboring child nodes of parentNode) {
  //    Compute its cost, f(x) = g(x) + h(x)
  //    If child node isn’t on Open or Closed list, put it on Open List.
  //    Else if child node is on Open or Closed List, AND this new one is cheaper,
  //    then take the old expensive one off both lists and put this new
  //    cheaper one on the Open List.
  //  }

  while (!openList.empty())
  {
    // Pop the cheapest node off the Open List
    Node* parentNode = openList.pop();

    // If parentNode is the Goal Node, then path found
    if (parentNode == goalNode)
    {
      // Build the path from start to goal
      Node* current = parentNode;
      while (current != nullptr)
      {
        request.path.push_back(terrain->get_world_position(current->gridPos));
        current = current->parent;
      }
      std::reverse(request.path.begin(), request.path.end()); // Reverse to get start to goal order
      return PathResult::COMPLETE;
    }

    // Place parentNode on the Closed List
    parentNode->onClosed = List::eClosed;

    // Optionally color the closed node for debugging
    if (request.settings.debugColoring)
    {
      terrain->set_color(parentNode->gridPos, Colors::Yellow);
    }

    std::vector<Node*> neighbors;
    FindValidNeighbors(parentNode, neighbors);

    // For all neighboring child nodes of parentNode
    for (const auto& neighbor : terrain->get_neighbors(parentNode->gridPos))
    {
      Node& childNode = grid[neighbor.row][neighbor.col];
      // Skip if child node is already on Closed List
      if (childNode.onClosed == List::eClosed)
        continue;
      // Calculate cost f(x) = g(x) + h(x)
      float tentativeCost = parentNode->givenCost + terrain->get_movement_cost(parentNode->gridPos, childNode.gridPos);
      float heuristicCost = CalculateHeuristicCost(request.settings.heuristic, childNode, endNode);
      float finalCost = tentativeCost + heuristicCost;
      // If child node isn’t on Open or Closed list, put it on Open List.
      if (childNode.onClosed == List::eUnvisited && openList.find(&childNode) == openList.end())
      {
        childNode.givenCost = tentativeCost;
        childNode.finalCost = finalCost;
        childNode.parent = parentNode;
        childNode.onClosed = List::eOpen;
        openList.push(&childNode);
      }
      // Else if child node is on Open or Closed List, AND this new one is cheaper,
      // then take the old expensive one off both lists and put this new cheaper one on the Open List.
      else if (openList.find(&childNode) != openList.end() && tentativeCost < childNode.givenCost)
      {
        childNode.givenCost = tentativeCost;
        childNode.finalCost = finalCost;
        childNode.parent = parentNode;
      }

      // If taken too much time this frame(or if request.settings.singleStep == true),
      //  abort search for now and resume next frame(return PathResult::PROCESSING).
  }

  /*
      This is where you handle pathing requests, each request has several fields:

      start/goal - start and goal world positions
      path - where you will build the path upon completion, path should be
          start to goal, not goal to start
      heuristic - which heuristic calculation to use
      weight - the heuristic weight to be applied
      newRequest - whether this is the first request for this path, should generally
          be true, unless single step is on

      smoothing - whether to apply smoothing to the path
      rubberBanding - whether to apply rubber banding
      singleStep - whether to perform only a single A* step
      debugColoring - whether to color the grid based on the A* state:
          closed list nodes - yellow
          open list nodes - blue

          use terrain->set_color(row, col, Colors::YourColor);
          also it can be helpful to temporarily use other colors for specific states
          when you are testing your algorithms

      method - which algorithm to use: A*, Floyd-Warshall, JPS+, or goal bounding,
          will be A* generally, unless you implement extra credit features

      The return values are:
          PROCESSING - a path hasn't been found yet, should only be returned in
              single step mode until a path is found
          COMPLETE - a path to the goal was found and has been built in request.path
          IMPOSSIBLE - a path from start to goal does not exist, do not add start position to path
  */

  // WRITE YOUR CODE HERE


  // Just sample code, safe to delete
  GridPos start = terrain->get_grid_position(request.start);
  GridPos goal = terrain->get_grid_position(request.goal);
  terrain->set_color(start, Colors::Orange);
  terrain->set_color(goal, Colors::Orange);
  request.path.push_back(request.start);
  request.path.push_back(request.goal);
  return PathResult::COMPLETE;
}

void AStarPather::ClearGrid()
{
  // Loop through each node on the grid
  for (size_t y = 0; y < HEIGHT; ++y)
  {
    for (size_t x = 0; x < WIDTH; ++x)
    {
      Node& tile = grid[y][x];

      tile.parent = nullptr;
      tile.givenCost = 0.0f;
      tile.finalCost = 0.0f;
      tile.onClosed = List::eUnvisited;
    }
  }
}

float AStarPather::CalculateHeuristicCost(const Heuristic& heuristic, const Node& curr, const Node& end)
{
  // Calculate difference between nodes
  float xDiff = fabsf(curr.gridPos.col - end.gridPos.col);
  float yDiff = fabsf(curr.gridPos.row - end.gridPos.row);

  float distance = 0.0f;

  switch (heuristic)
  {
    // min(xDiff, yDiff) * sqrt(2) + max(xDiff, yDiff) – min(xDiff, yDiff)
  case Heuristic::OCTILE:
    distance = fminf(xDiff, yDiff) * sqrtf(2.0f) + fmaxf(xDiff, yDiff) - fminf(xDiff, yDiff);
    break;

    // max(xDiff, yDiff) 
  case Heuristic::CHEBYSHEV:
    distance = fmaxf(xDiff, yDiff);
    break;

    // xDiff + yDiff
  case Heuristic::MANHATTAN:
    distance = xDiff + yDiff;
    break;

    // Inconsistent Heuristic code given by Professor Rabin
  case Heuristic::INCONSISTENT:
    if ((curr.gridPos.row + curr.gridPos.col) % 2 <= 0)
    {
      return 0;
    }
    // PURPOSEFULLY FALL THROUGH TO EUCLIDEAN DISTANCE
    __fallthrough;

    // sqrt(xDiff2 + yDiff2) 
  case Heuristic::EUCLIDEAN:
    distance = sqrtf((xDiff * xDiff) + (yDiff * yDiff));
    break;

  default:
    // NEED TO SAY SOMETHING ABOUT INVALID HEURISTIC
    break;
  }

  return distance;
}

float AStarPather::CalculateGivenCost(const Node& curr)
{
  return 0.0f;
}

// GITHUB COPILOT GENERATION
void AStarPather::FindValidNeighbors(const Node* curr, std::vector<Node*>& neighbors)
{
  // Look at the grid position of this node
  GridPos pos = curr.gridPos;

  std::vector<GridPos> invalidPositions;

  // Check all 8 possible neighbors (N, NE, E, SE, S, SW, W, NW)
  for (int rowOffset = -1; rowOffset <= 1; ++rowOffset)
  {
    for (int colOffset = -1; colOffset <= 1; ++colOffset)
    {
      // Skip the current node itself
      if (rowOffset == 0 && colOffset == 0)
      {
        continue;
      }

      // Calculate the neighbor's position
      GridPos neighborPos{ pos.row + rowOffset, pos.col + colOffset };

      if (terrain->is_valid_grid_position(neighborPos))
      {
        invalidPositions.push_back(neighborPos);
        continue; // Skip invalid grid positions
      }

      // Check if the neighbor is a wall
      if (terrain->is_wall(neighborPos))
      {
        // Need to know if this is a N, W, S, E neighbor
        if (abs(rowOffset) == abs(colOffset)
        {
          // Diagonal neighbors can not be valid
          invalidPositions.push_back(neighborPos);
        }

        continue; // Skip walls or invalid positions
      }

      Node& neighborNode = grid[neighborPos.row][neighborPos.col];
      // Only add valid neighbors that are not on the closed list
      if (neighborNode.onClosed != List::eClosed)
      {
        // Not on the closed list, initialize the data within this node
        neighborNode.gridPos = neighborPos;

        neighbors.push_back(&neighborNode);
      }

    }
  }
}
