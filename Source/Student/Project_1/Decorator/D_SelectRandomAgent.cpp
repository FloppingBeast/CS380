/*********************************************************************
 * file:   D_SelectRandomAgent.cpp
 * author: lawrence.winters (lawrence.winters@digipen.edu)
 * date:   June 3, 2025
 * Copyright © 2024 DigiPen (USA) Corporation.
 * 
 * brief: 
 *********************************************************************/
#include <pch.h>
#include "D_SelectRandomAgent.h"

D_SelectRandomAgent::D_SelectRandomAgent() : rndAgent(nullptr)
{
}

void D_SelectRandomAgent::on_enter()
{
  // Select a random agent from the list of agents
  const std::vector<Agent*>& allAgents = agents->get_all_agents();

  // Remove the current agent from the list to avoid selecting itself
  std::vector<Agent*> avaliableAgents = allAgents;
  auto itr = std::remove(avaliableAgents.begin(), avaliableAgents.end(), agent);
  avaliableAgents.erase(itr, avaliableAgents.end());
  
  // Set the random agent on the blackboard
  Blackboard& blackboard = agent->get_blackboard();
  if (!avaliableAgents.empty())
  {
    // Select a random agent from the list
    int rndIndex = RNG::range<int>(0, static_cast<int>(avaliableAgents.size() - 1));
    rndAgent = avaliableAgents[rndIndex];

    // Set the selected agent in the blackboard
    blackboard.set_value("savedAgent", rndAgent);
  }
  else
  {
    blackboard.set_value("savedAgent", nullptr);
  }

  BehaviorNode::on_enter();
}

void D_SelectRandomAgent::on_update(float dt)
{
  // Run the child node
  BehaviorNode* child = children.front();

  child->tick(dt);

  // assume same status as child
  set_status(child->get_status());
  set_result(child->get_result());
}
