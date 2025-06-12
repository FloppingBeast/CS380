/*********************************************************************
 * file:   C_FlipFlop.cpp
 * author: lawrence.winters (lawrence.winters@digipen.edu)
 * date:   June 3, 2025
 * Copyright © 2024 DigiPen (USA) Corporation.
 * 
 * brief:
 *********************************************************************/
#include <pch.h>
#include "C_FlipFlop.h"

C_FlipFlop::C_FlipFlop() : index(0)
{
}

void C_FlipFlop::on_update(float dt)
{
  // saves the current index so we can restore it if flip flop is interrupted
  BehaviorNode* currentNode = children[index];
  currentNode->tick(dt);

  if (currentNode->failed() == true)
  {
    // move to the next node
    ++index;
    if (index >= children.size())
    {
      index = 0; // wrap around to the first child
      on_failure();
    }
  }
  else if (currentNode->succeeded() == true)
  {
    // move to the next node
    index = (index + 1) % children.size();
    on_success();
  }
}
