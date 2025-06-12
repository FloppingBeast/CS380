/*********************************************************************
 * file:   L_SpinCounterClockwise.h
 * author: lawrence.winters (lawrence.winters@digipen.edu)
 * date:   June 4, 2025
 * Copyright © 2024 DigiPen (USA) Corporation.
 * 
 * brief: 
 *********************************************************************/
#pragma once
#include "BehaviorNode.h"

class L_SpinCounterClockwise : public BaseNode<L_SpinCounterClockwise>
{
protected:
  virtual void on_enter() override;
  virtual void on_update(float dt) override;

private:
  float targetRotation;

  // is near target rotation
  bool is_near_target_rotation(float targetRotation, float currentRotation, float threshold = 0.1f) const
  {
    return std::abs(targetRotation - currentRotation) < threshold;
  }
};