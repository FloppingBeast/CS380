/*********************************************************************
 * file:   L_MoveToSavedAgent.cpp
 * author: lawrence.winters (lawrence.winters@digipen.edu)
 * date:   June 4, 2025
 * Copyright © 2024 DigiPen (USA) Corporation.
 * 
 * brief: 
 *********************************************************************/
#include <pch.h>
#include "L_MoveToSavedAgent.h"
#include "Agent/BehaviorAgent.h"

void L_MoveToSavedAgent::on_enter()
{
  // set animation, speed, etc
  agent->set_movement_speed(20.0f);

  Blackboard& blackboard = agent->get_blackboard();

  // Agent should have a saved agent in the blackboard from decorator
  Agent* savedAgent = blackboard.get_value<Agent*>("savedAgent");

  // Get the point where the saved agent was last seen
  if (savedAgent == nullptr)
  {
    on_failure();
    return;
  }

  targetPosition = savedAgent->get_position();

  BehaviorNode::on_leaf_enter();
}

void L_MoveToSavedAgent::on_update(float dt)
{
  const auto result = agent->move_toward_point(targetPosition, dt);

  if (result == true)
  {
    on_success();
  }

  display_leaf_text();
}