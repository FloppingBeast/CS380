/*********************************************************************
 * file:   L_FollowAgent.h
 * author: lawrence.winters (lawrence.winters@digipen.edu)
 * date:   June 4, 2025
 * Copyright © 2024 DigiPen (USA) Corporation.
 * 
 * brief: 
 *********************************************************************/
#pragma once
#include "BehaviorNode.h"

class L_FollowAgent : public BaseNode<L_FollowAgent>
{
public:
  L_FollowAgent();

protected:
  float timer = 0.0f;
  Vec3 moveAwayPos;
  float newYaw = 0.0f;

  virtual void on_enter() override;
  virtual void on_update(float dt) override;
};