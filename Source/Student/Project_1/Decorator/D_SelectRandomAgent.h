/*********************************************************************
 * file:   D_SelectRandomAgent.h
 * author: lawrence.winters (lawrence.winters@digipen.edu)
 * date:   June 3, 2025
 * Copyright © 2024 DigiPen (USA) Corporation.
 * 
 * brief: 
 *********************************************************************/
#pragma once
#include "BehaviorNode.h"

class D_SelectRandomAgent : public BaseNode<D_SelectRandomAgent>
{
public:
  D_SelectRandomAgent();

protected:
  Agent* rndAgent;

  virtual void on_enter() override;
  virtual void on_update(float dt) override;
};