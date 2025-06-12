/*********************************************************************
 * file:   L_Jump.h
 * author: lawrence.winters (lawrence.winters@digipen.edu)
 * date:   June 4, 2025
 * Copyright © 2024 DigiPen (USA) Corporation.
 * 
 * brief: 
 *********************************************************************/
#pragma once
#include "BehaviorNode.h"

class L_Jump : public BaseNode<L_Jump>
{
public:
  L_Jump();

protected:
  Vec3 targetPosition;
  Vec3 originalHeight;
  float originalSpeed;

  bool upward = false;
  bool downward = false;

  virtual void on_enter() override;
  virtual void on_update(float dt) override;
  virtual void on_exit() override;
};