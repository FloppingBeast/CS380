/*********************************************************************
 * file:   L_SpinCounterClockwise.cpp
 * author: lawrence.winters (lawrence.winters@digipen.edu)
 * date:   June 3, 2025
 * Copyright © 2024 DigiPen (USA) Corporation.
 * 
 * brief: 
 *********************************************************************/
#include <pch.h>
#include "L_SpinCounterClockwise.h"
#include "Agent/BehaviorAgent.h"

void L_SpinCounterClockwise::on_enter()
{
  // Get the agents current rotation
  const float& currRotation = agent->get_yaw();

  // Calculate the target point based on the starting rotation
  targetRotation = currRotation + (2.0f * PI); // Spin 1 full rotation clockwise

  // Set rotation speed in the blackboard (MAYBE DO THIS IN SETUP?)
  Blackboard& blackboard = agent->get_blackboard();
  blackboard.set_value("rotSpeed", 3.0f);

  BehaviorNode::on_leaf_enter();
}

void L_SpinCounterClockwise::on_update(float dt)
{
  // Get the rotation speed from the blackboard
  Blackboard& blackboard = agent->get_blackboard();
  const char* rotationSpeedKey = "rotSpeed";

  // Get the agents current rotation
  float currRotation = agent->get_yaw();

  // Calculate an amount to rotate based on the time delta
  const float newRotation = currRotation + dt * blackboard.get_value<float>(rotationSpeedKey);

  // Update the agent's rotation
  agent->set_yaw(newRotation);

  if (is_near_target_rotation(targetRotation, newRotation))
  {
    on_success();
  }

  display_leaf_text();
}
