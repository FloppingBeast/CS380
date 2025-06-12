/*********************************************************************
 * file:   L_Scale.h
 * author: lawrence.winters (lawrence.winters@digipen.edu)
 * date:   June 4, 2025
 * Copyright © 2024 DigiPen (USA) Corporation.
 * 
 * brief: 
 *********************************************************************/
#pragma once
#include "BehaviorNode.h"

class L_Scale : public BaseNode<L_Scale>
{
protected:
  virtual void on_enter() override;
  virtual void on_update(float dt) override;

private:
  Vec3 targetScale;
  Vec3 scaleFactor;

  // is near target rotation
  bool is_near_target_scale(Vec3 targetScale, Vec3 currentScale, Vec3 threshold = Vec3(0.1f)) const
  {
    // Only need to compare one axis since uniform scaling is assumed
    return std::abs(targetScale.x - currentScale.x) < threshold.x;
  }
};