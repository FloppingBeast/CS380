/*********************************************************************
 * file:   D_SkipFirstTime.h
 * author: lawrence.winters (lawrence.winters@digipen.edu)
 * date:   June 4, 2025
 * Copyright © 2024 DigiPen (USA) Corporation.
 * 
 * brief: 
 *********************************************************************/
#pragma once
#include "BehaviorNode.h"

class D_SkipFirstTime : public BaseNode<D_SkipFirstTime>
{
public:
  D_SkipFirstTime();

protected:
  bool shouldRun;

  virtual void on_update(float dt) override;
  virtual void on_exit() override;
};