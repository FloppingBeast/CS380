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

  // Hook up the callback for map changes
  Callback cb = std::bind(&AStarPather::OnMapChange, this);
  Messenger::listen_for_message(Messages::MAP_CHANGE, cb);

  // If you want to precompute once for the initial map as well:
  //OnMapChange();

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
  // Check to see if new path has been requested
  if (request.newRequest)
  {
    NewPathRequest(request);
  }

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


      // Rubberbanding the path if requested
      if (request.settings.rubberBanding)
      {
        RubberbandPath(request);
      }

      // Smoothing the path if requested
      if (request.settings.smoothing)
      {
        SmoothPath(request);
      }

      return PathResult::COMPLETE;
    }

    // Place parentNode on the Closed List
    parentNode->onClosed = List::eClosed;

    // Optionally color the closed node for debugging
    if (request.settings.debugColoring)
    {
      terrain->set_color(parentNode->gridPos, Colors::Yellow);
    }

    // For all neighboring child nodes of parentNode
    for (Node* neighbor : precomputedNeighbors[index_for(parentNode->gridPos)])
    {
      // Calculate cost f(x) = g(x) + h(x)
      float tempGivenCost = parentNode->givenCost + CalculateGivenCost(parentNode, neighbor);
      float heuristicCost = CalculateHeuristicCost(request.settings.heuristic, *neighbor, *goalNode);
      float finalCost = tempGivenCost + (heuristicCost * request.settings.weight);

      // If child node isn’t on Open or Closed list, put it on Open List.
      if (neighbor->onClosed == List::eUnvisited)
      {
        neighbor->givenCost = tempGivenCost;
        neighbor->finalCost = finalCost;
        neighbor->parent = parentNode;
        neighbor->onClosed = List::eOpen;

        // Color the open node for debugging
        if (request.settings.debugColoring)
        {
          terrain->set_color(neighbor->gridPos, Colors::Blue);
        }

        openList.push(neighbor);
      }
      // Else if child node is on Open or Closed List, AND this new one is cheaper,
      else if ((neighbor->onClosed == List::eOpen || neighbor->onClosed == List::eClosed) && tempGivenCost < neighbor->givenCost)
      {
        neighbor->givenCost = tempGivenCost;
        neighbor->finalCost = finalCost;
        neighbor->parent = parentNode;

        // then take the old expensive one off both lists and put this new cheaper one on the Open List.
        if (neighbor->onClosed == List::eClosed)
        {
          neighbor->onClosed = List::eOpen; // re-open it

          // Color the open node for debugging
          if (request.settings.debugColoring)
          {
            terrain->set_color(neighbor->gridPos, Colors::Blue);
          }
        }

        // In an optimized structure, you'd also update its position
        // in the priority queue, but not needed for vector.
        // Always push it into the open list again
        // Even if it's already in the open list — let your openList implementation handle duplicates or priority updates.
        openList.push(neighbor);
      }
    }

    if (request.settings.singleStep)
    {
      // If taken too much time this frame, abort search for now and resume next frame
      // (return PathResult::PROCESSING).
      return PathResult::PROCESSING;
    }
  }

  return PathResult::IMPOSSIBLE;
}

void AStarPather::ClearGrid()
{
  // Loop through each node on the grid
  for (int y = 0; y < HEIGHT; ++y)
  {
    for (int x = 0; x < WIDTH; ++x)
    {
      Node& tile = grid[index_for(y, x)];

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
  float xDiff = fabsf(static_cast<float>(curr.gridPos.col - end.gridPos.col));
  float yDiff = fabsf(static_cast<float>(curr.gridPos.row - end.gridPos.row));

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
    if ((curr.gridPos.row + curr.gridPos.col) % 2 > 0)
    {
      distance = sqrtf((xDiff * xDiff) + (yDiff * yDiff));
    }
    break;


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

float AStarPather::CalculateGivenCost(const Node* curr, const Node* next)
{
  // Find distance between the current node and the next node
  int xDiff = curr->gridPos.col - next->gridPos.col;
  int yDiff = curr->gridPos.row - next->gridPos.row;
  
  // If the next node is diagonal, then the cost is sqrt(2)
  return sqrtf(static_cast<float>((xDiff * xDiff) + (yDiff * yDiff)));
}

void AStarPather::NewPathRequest(PathRequest& request)
{
  // Clear the grid
  ClearGrid();

  // Clear the open list
  openList.clear();

  // Push Start Node onto the Open List 
  GridPos startPos = terrain->get_grid_position(request.start);
  Node& startNode = grid[index_for(startPos.row, startPos.col)];
  startNode.gridPos = startPos;
  startNode.onClosed = List::eOpen;

  if (request.settings.debugColoring)
  {
    terrain->set_color(startPos, Colors::Blue);
  }

  startNode.parent = nullptr;
  openList.push(&startNode);

  // Update the end node position
  GridPos endPos = terrain->get_grid_position(request.goal);
  Node& endNode = grid[index_for(endPos.row,endPos.col)];
  endNode.gridPos = endPos;
  goalNode = &endNode;

  // Calculate final cost f(x) = g(x) + h(x).
  startNode.givenCost = 0.0f;
  startNode.finalCost = startNode.givenCost + (CalculateHeuristicCost(request.settings.heuristic, startNode, endNode) * request.settings.weight);
}

void AStarPather::RubberbandPath(PathRequest& request)
{
  // Convert the list to a vector for easier manipulation
  std::vector<Vec3> pathVector(request.path.begin(), request.path.end());

  // Take three points at a time from the list
  int pathSize = static_cast<int>(pathVector.size());
  for (int i = (pathSize - 1); (i - 2) >= 0; --i)
  {
    Vec3 p1 = pathVector[i];
    Vec3 p2 = pathVector[i - 1];
    Vec3 p3 = pathVector[i - 2];

    // Create a square bounding box around the three points
    float minCol = std::min({ p1.x, p2.x, p3.x });
    float maxCol = std::max({ p1.x, p2.x, p3.x });
    float minRow = std::min({ p1.z, p2.z, p3.z });
    float maxRow = std::max({ p1.z, p2.z, p3.z });



    // Get grid positions for the bounding box
    GridPos minGridPos = terrain->get_grid_position(Vec3(minCol, 0.0f, minRow));
    GridPos maxGridPos = terrain->get_grid_position(Vec3(maxCol, 0.0f, maxRow));

    // Go from minGridPos to maxGridPos to form bounding box
    bool wall = false;
    for (int row = minGridPos.row; row <= maxGridPos.row; ++row)
    {
      for (int col = minGridPos.col; col <= maxGridPos.col; ++col)
      {
        // Get the grid position
        GridPos gridPos = { row, col };

        // Check if the grid position is valid and not a wall
        if (terrain->is_wall(gridPos))
        {
          wall = true;
          break;
        }
      }

      if (wall)
      {
        break; // Exit early if a wall is found
      }
    }

    // If no wall is found in bounding box, remove the middle point
    if (!wall)
    {
      // Erase the middle point
      pathVector.erase(pathVector.begin() + (i - 1));
    }
  }

  // Add the new points to the smoothed path
  WaypointList rubberbandPath(pathVector.begin(), pathVector.end());

  // Replace the original path with the smoothed path
  request.path.clear();
  request.path.insert(request.path.end(), rubberbandPath.begin(), rubberbandPath.end());
}

void AStarPather::SmoothPath(PathRequest& request)
{
  // Convert the list to a vector for easier manipulation
  std::vector<Vec3> pathVector(request.path.begin(), request.path.end());

  // Rubberbanding had been performed, may need to add points back in
  if (request.settings.rubberBanding)
  {
    // Check to see how far about grid squares are in world coords
    Vec3 pos0 = terrain->get_world_position(GridPos{ 0, 0 });
    Vec3 pos1 = terrain->get_world_position(GridPos{ 0, 1 });

    Vec3 tileDistance = pos1 - pos0;

    // Get magnitude of the tile distance
    float tileMagnitude = std::sqrt(tileDistance.x * tileDistance.x + tileDistance.z * tileDistance.z);
    float maxDistance = tileMagnitude * 1.5f; // Set max distance to 1.5 tiles

    for (size_t i = 0; i < pathVector.size() - 1;)
    {
      Vec3 current = pathVector[i];
      Vec3 next = pathVector[i + 1];

      // Get the grid positions of the current and next points
      GridPos currentSquare = terrain->get_grid_position(current);
      GridPos nextSquare = terrain->get_grid_position(next);

      Vec3 distanceVec = next - current;

      float dist = std::sqrt((distanceVec.x * distanceVec.x) + (distanceVec.z * distanceVec.z));

      if (dist > maxDistance)
      {
        // Calculate a point in the middle of the two points
        Vec3 midPoint = (current + next) * 0.5f;

        // Add the midPoint to the pathVector
        pathVector.insert(pathVector.begin() + (i + 1), midPoint);
      }
      else
      {
        // We can increment to the next point
        ++i;
      }
    }
  }

  // Start to smooth the path using Catmull-Rom Spline
  float pathSize = static_cast<int>(pathVector.size());
  std::vector<Vec3> newPath;

  // First need to get  four points to use for Catmull-Rom Spline (if not enough points, just use the first and last)
  for (size_t i = 0; (i + 2) <= pathSize; ++i)
  {
    Vec3 p0, p1, p2, p3;

    if (i == 0)
    {
      p0 = pathVector[i];
      p1 = p0;
      p2 = pathVector[i + 1];
      p3 = pathVector[i + 2];
    }
    else if (i + 2 == pathSize)
    {
      p0 = pathVector[i - 1];
      p1 = pathVector[i];
      p2 = pathVector[i + 1];
      p3 = pathVector[i + 1]; // Use the last point again
    }
    else
    {
      p0 = pathVector[i - 1];
      p1 = pathVector[i];
      p2 = pathVector[i + 1];
      p3 = pathVector[i + 2];
    }

    newPath.push_back(p1);

    // Perform Catmull-Rom Spline interpolation on t values of .25, .5, and .75
    for (float t = 0.25f; t <= .76f; t += 0.25f)
    {
      // Catmull-Rom Spline
      Vec3 newPoint = Vec3();
      Vec3::CatmullRom(p0, p1, p2, p3, t, newPoint);

      // Insert the new points list into the path after point i in std::list
      newPath.push_back(newPoint);
    }

    if ((i + 2) == pathSize)
    {
      // If we are at the end of the path, add the last point
      newPath.push_back(pathVector[i + 1]);
    }
  }

  // Add the new points to the smoothed path
  WaypointList smoothedPath(newPath.begin(), newPath.end());

  // Replace the original path with the smoothed path
  request.path.clear();
  request.path.insert(request.path.end(), smoothedPath.begin(), smoothedPath.end());
}

void AStarPather::OnMapChange()
{
  for (int row = 0; row < HEIGHT; ++row)
  {
    for (int col = 0; col < WIDTH; ++col)
    {
      // Initialize the grid node
      Node& node = grid[index_for(row,col)];
      node.gridPos = { row, col };

      int idx = index_for(row, col);
      precomputedNeighbors[idx].clear();

      if (!terrain->is_valid_grid_position(node.gridPos) || terrain->is_wall(node.gridPos))
      {
        continue;
      }

      for (int dy = -1; dy <= 1; ++dy)
      {
        for (int dx = -1; dx <= 1; ++dx)
        {
          // Itself
          if (dx == 0 && dy == 0)
          {
            continue;
          }

          int ny = row + dy;
          int nx = col + dx;

          GridPos neighborPos{ ny, nx };

          if (!terrain->is_valid_grid_position(neighborPos) || terrain->is_wall(neighborPos))
          {
            continue;
          }

          if (abs(dx) + abs(dy) == 2)
          {
            if (terrain->is_wall({ row, col + dx }) || terrain->is_wall({ row + dy, col }))
            {
              continue;
            }
          }

          precomputedNeighbors[idx].push_back(&grid[index_for(ny, nx)]);
        }
      }
    }
  }
}
