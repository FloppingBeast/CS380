#include <pch.h>
#include "L_PlaySound.h"

void L_PlaySound::on_enter()
{
	// Look at the agent name and choose sound based on that
	audioManager->PlaySoundEffect(L"Assets\\Audio\\TreeVegetation.wav");
	BehaviorNode::on_leaf_enter();
	on_success();
}