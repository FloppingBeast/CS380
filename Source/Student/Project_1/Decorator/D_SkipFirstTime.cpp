/*********************************************************************
 * file:   D_SkipFirstTime.cpp
 * author: lawrence.winters (lawrence.winters@digipen.edu)
 * date:   June 4, 2025
 * Copyright © 2024 DigiPen (USA) Corporation.
 * 
 * brief: 
 *********************************************************************/
#include <pch.h>
#include "D_SkipFirstTime.h"

D_SkipFirstTime::D_SkipFirstTime() : shouldRun(false)
{
}

void D_SkipFirstTime::on_update(float dt)
{
  // Checks to see if we should run the child node
  if (shouldRun)
  {
    // Perform the child node
    BehaviorNode* child = children.front();

    child->tick(dt);

    // assume same status as child
    if (child->succeeded() == true)
    {
      on_success();
    }
    else if (child->failed() == true)
    {
      on_failure();
    }
  }
  else
  {
    BehaviorNode::on_failure();
  }
}

void D_SkipFirstTime::on_exit()
{
  // Flip the shouldRun flag
  shouldRun = !shouldRun;
}