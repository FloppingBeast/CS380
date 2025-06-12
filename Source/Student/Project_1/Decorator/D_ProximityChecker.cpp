/*********************************************************************
 * file:   D_ProximityChecker.cpp
 * author: lawrence.winters (lawrence.winters@digipen.edu)
 * date:   June 4, 2025
 * Copyright © 2024 DigiPen (USA) Corporation.
 * 
 * brief: 
 *********************************************************************/
#include <pch.h>
#include "D_ProximityChecker.h"

D_ProximityChecker::D_ProximityChecker() : radius(10.0f)
{
}

void D_ProximityChecker::on_enter()
{
  // Get all the agents from the agent manager
  const std::vector<Agent*>& agentList = agents->get_all_agents();

  // Check if any agent is within the specified proximity
  for (Agent* other : agentList)
  {
    // Check if the agent is the same as the one this decorator is attached to
    if (other == agent)
    {
      continue;
    }

    // Calculate the distance between the 
    float distance = DirectX::SimpleMath::Vector3::Distance(other->get_position(), agent->get_position());

    if (distance <= radius)
    {
      // If an agent is within proximity, set the status to success
      BehaviorNode::on_enter();

      // Save the agent that is within proximity
      Blackboard& blackboard = agent->get_blackboard();
      blackboard.set_value("savedAgent", other);

      return;
    }
  }
  
  // If no agents are within proximity, set the status to failure
  BehaviorNode::on_failure();
}

void D_ProximityChecker::on_update(float dt)
{
  // Run the child node
  BehaviorNode* child = children.front();

  child->tick(dt);

  // Assume same status as child
  set_status(child->get_status());
  set_result(child->get_result());
}
