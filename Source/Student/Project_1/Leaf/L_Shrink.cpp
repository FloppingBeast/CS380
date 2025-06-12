/*********************************************************************
 * file:   L_Shrink.cpp
 * author: lawrence.winters (lawrence.winters@digipen.edu)
 * date:   June 4, 2025
 * Copyright © 2024 DigiPen (USA) Corporation.
 * 
 * brief: 
 *********************************************************************/
#include <pch.h>
#include "L_Shrink.h"
#include "Agent/BehaviorAgent.h"

void L_Shrink::on_enter()
{
  // Get a random value to scale the agent by
  float randFloat = RNG::range<float>(0.5f, 0.8f);

  // Get the agent's current scale
  Vec3 currScale = agent->get_scaling();

  // Calculate the target scale
  targetScale = currScale * randFloat;

  // Calculate a scale factor based on the difference between these vectors
  scaleFactor = (currScale - targetScale) / 2.0f;

  BehaviorNode::on_leaf_enter();
}

void L_Shrink::on_update(float dt)
{
  // Get the agents current scale
  Vec3 currScale = agent->get_scaling();

  // Calculate an amount to rotate based on the time delta
  Vec3 newScale = currScale - dt * scaleFactor;

  // Update the agent's rotation
  agent->set_scaling(newScale);

  // Determine threshold based on the target scale
  Vec3 threshold = scaleFactor / 5.0f;

  if (is_near_target_scale(targetScale, newScale, threshold))
  {
    on_success();
  }

  display_leaf_text();
}