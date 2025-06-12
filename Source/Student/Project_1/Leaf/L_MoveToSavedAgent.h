/*********************************************************************
 * file:   L_MoveToSavedAgent.h
 * author: lawrence.winters (lawrence.winters@digipen.edu)
 * date:   June 4, 2025
 * Copyright © 2024 DigiPen (USA) Corporation.
 * 
 * brief: 
 *********************************************************************/
#pragma once
#include "BehaviorNode.h"

class L_MoveToSavedAgent : public BaseNode<L_MoveToSavedAgent>
{
protected:
  virtual void on_enter() override;
  virtual void on_update(float dt) override;

private:
  Vec3 targetPosition;
};