/*********************************************************************
 * file:   L_FollowAgent.cpp
 * author: lawrence.winters (lawrence.winters@digipen.edu)
 * date:   June 4, 2025
 * Copyright © 2024 DigiPen (USA) Corporation.
 * 
 * brief: 
 *********************************************************************/
#include <pch.h>
#include "L_FollowAgent.h"
#include "Agent/BehaviorAgent.h"

L_FollowAgent::L_FollowAgent() : timer(0.0f)
{
}

void L_FollowAgent::on_enter()
{
  timer = RNG::range<float>(7.5f, 10.0f);

  moveAwayPos = Vec3(0, 0, 0);

  BehaviorNode::on_leaf_enter();
}

void L_FollowAgent::on_update(float dt)
{
  // Check the timer for completion
  timer -= dt;

  if (timer <= 0.0f)
  {
    on_success();

    return;
  }

  Blackboard& blackboard = agent->get_blackboard();
  Agent* leader = blackboard.get_value<Agent*>("savedAgent");

  if (!leader)
  {
    on_failure();
    return;
  }

  // Check the distance to the leader
  Vec3 leaderPos = leader->get_position();
  float distance = SimpleMath::Vector3::Distance(leaderPos, agent->get_position());
  if (distance > 20.0f)
  {
    const auto result = agent->move_toward_point(leaderPos , dt);
  }
  else if (distance < 10.0f)
  {
    // Calculate a move-away position
    if (moveAwayPos == Vec3(0, 0, 0))
    {
      // Move the object away from the leader
      Vec3 direction = agent->get_position() - leaderPos;
      direction.Normalize();

      // angle the agent to face the leader
      newYaw = atan2(-direction.x, -direction.z); // radians

      moveAwayPos = agent->get_position() + direction * 5.0f; // Move away by 5 units
    }

    bool result = agent->move_toward_point(moveAwayPos, dt);

    if (result)
    {
      agent->set_yaw(newYaw); // Set the yaw to face the leader
    }
  }

  display_leaf_text();
}