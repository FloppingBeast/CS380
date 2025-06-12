/*********************************************************************
 * file:   D_ProximityChecker.h
 * author: lawrence.winters (lawrence.winters@digipen.edu)
 * date:   June 4, 2025
 * Copyright © 2024 DigiPen (USA) Corporation.
 * 
 * brief: 
 *********************************************************************/
#pragma once
#include "BehaviorNode.h"

class D_ProximityChecker : public BaseNode<D_ProximityChecker>
{
public:
  D_ProximityChecker();

protected:
  float radius = 0.0f;

  virtual void on_enter() override;
  virtual void on_update(float dt) override;
};