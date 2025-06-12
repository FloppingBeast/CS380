/*********************************************************************
 * file:   L_Jump.cpp
 * author: lawrence.winters (lawrence.winters@digipen.edu)
 * date:   June 4, 2025
 * Copyright © 2024 DigiPen (USA) Corporation.
 * 
 * brief: 
 *********************************************************************/
#include <pch.h>
#include "L_Jump.h"

L_Jump::L_Jump() : targetPosition(Vec3(0.0f)), originalHeight(Vec3(0.0f)), originalSpeed(0.0f), upward(false), downward(false)
{
}

void L_Jump::on_enter()
{
  upward = false;
  downward = false;

  // Calculate the target jump height
  originalHeight = agent->get_position();

  float maxHeight = originalHeight.y + RNG::range(10.0f, 20.0f);
  targetPosition = Vec3(originalHeight.x, maxHeight, originalHeight.z);

  originalSpeed = agent->get_movement_speed();
  agent->set_movement_speed(50.0f); // Double the speed for the jump

  BehaviorNode::on_leaf_enter();
}

void L_Jump::on_update(float dt)
{
  if (!upward)
  {
    upward = agent->move_toward_point(targetPosition, dt);
  }
  else if (!downward)
  {
    downward = agent->move_toward_point(originalHeight, dt);
  }
  else
  {
    on_success();
  }

  display_leaf_text();
}

void L_Jump::on_exit()
{
  agent->set_movement_speed(originalSpeed); // Reset the speed after the jump
}