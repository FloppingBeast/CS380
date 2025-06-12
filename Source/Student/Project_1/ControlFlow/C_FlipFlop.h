/*********************************************************************
 * file:   C_FlipFlop.h
 * author: lawrence.winters (lawrence.winters@digipen.edu)
 * date:   June 4, 2025
 * Copyright © 2024 DigiPen (USA) Corporation.
 * 
 * brief: 
 *********************************************************************/
#pragma once
#include "BehaviorNode.h"

class C_FlipFlop : public BaseNode<C_FlipFlop>
{
public:
  C_FlipFlop();

protected:
  size_t index = 0;

  virtual void on_update(float dt) override;
};