#include <pch.h>
#include "Projects/ProjectOne.h"
#include "Agent/CameraAgent.h"

void ProjectOne::setup()
{
    // Create an agent with a different 3D model:
    // 1. (optional) Add a new 3D model to the framework other than the ones provided:
    //    A. Find a ".sdkmesh" model or use https://github.com/walbourn/contentexporter
    //       to convert fbx files (many end up corrupted in this process, so good luck!)
    //    B. Add a new AgentModel enum for your model in Agent.h (like the existing Man or Tree).
    // 2. Register the new model with the engine, so it associates the file path with the enum
    //    A. Here we are registering all of the extra models that already come in the package.
    Agent::add_model("Assets\\tree.sdkmesh", Agent::AgentModel::Tree);
    Agent::add_model("Assets\\bird.sdkmesh", Agent::AgentModel::Bird);
    Agent::add_model("Assets\\ball.sdkmesh", Agent::AgentModel::Ball);

    // 3. Create the agent, giving it the correct AgentModel type.
    auto tree = agents->create_behavior_agent("Tree", BehaviorTreeTypes::Tree, Agent::AgentModel::Tree);
    auto bird = agents->create_behavior_agent("Bird", BehaviorTreeTypes::Bird, Agent::AgentModel::Bird);
    auto ball = agents->create_behavior_agent("Ball", BehaviorTreeTypes::Ball, Agent::AgentModel::Ball);
    auto man = agents->create_behavior_agent("Man", BehaviorTreeTypes::Man, Agent::AgentModel::Man);

    // put the 4 agents in separate corners of the map
    bird->set_position(Vec3(100, 0, 100)); 
    tree->set_position(Vec3(50, 0, 50));
    ball->set_position(Vec3(100, 10, 0)); // move ball above ground
    man->set_position(Vec3(0, 0, 100));
    
    // Rescale necessary models to fit the map
    bird->set_scaling(Vec3(0.01f, 0.01f, 0.01f)); // Scale the bird down to half size
    ball->set_scaling(Vec3(0.5f, 0.5f, 0.5f)); // Scale the ball down to half size

    // Color objects
    tree->set_color(Vec3(0, 0.5, 0));
    bird->set_color(Vec3(0.5, 0.5, 1));
    ball->set_color(Vec3(1, 0.5, 0));
    man->set_color(Vec3(0, 0, 0));

    // Set pitch of the tree
    tree->set_pitch(PI / 2);

    // You can technically load any map you want, even create your own map file,
    // but behavior agents won't actually avoid walls or anything special, unless you code
    // that yourself (that's the realm of project 2)
    terrain->goto_map(0);

    // You can also enable the pathing layer and set grid square colors as you see fit.
    // Works best with map 0, the completely blank map

    // Camera position can be modified from this default
    auto camera = agents->get_camera_agent();
    camera->set_position(Vec3(-62.0f, 70.0f, terrain->mapSizeInWorld * 0.5f));
    camera->set_pitch(0.610865); // 35 degrees

    // Sound control (these sound functions can be kicked off in a behavior tree node - see the example in L_PlaySound.cpp)
    audioManager->SetVolume(0.5f);
}