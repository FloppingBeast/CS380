#pragma once

// Include all node headers in this file

// Example Control Flow Nodes
#include "ControlFlow/C_ParallelSequencer.h"
#include "ControlFlow/C_RandomSelector.h"
#include "ControlFlow/C_Selector.h"
#include "ControlFlow/C_Sequencer.h"

// Student Control Flow Nodes
#include "ControlFlow/C_FlipFlop.h"

// Example Decorator Nodes
#include "Decorator/D_Delay.h"
#include "Decorator/D_InvertedRepeater.h"
#include "Decorator/D_RepeatFourTimes.h"

// Student Decorator Nodes
#include "Decorator/D_SelectRandomAgent.h"
#include "Decorator/D_ProximityChecker.h"
#include "Decorator/D_SkipFirstTime.h"

// Example Leaf Nodes
#include "Leaf/L_CheckMouseClick.h"
#include "Leaf/L_Idle.h"
#include "Leaf/L_MoveToFurthestAgent.h"
#include "Leaf/L_MoveToMouseClick.h"
#include "Leaf/L_MoveToRandomPosition.h"
#include "Leaf/L_PlaySound.h"

// Student Leaf Nodes
#include "Leaf/L_SpinClockwise.h"
#include "Leaf/L_SpinCounterClockwise.h"
#include "Leaf/L_MoveToSavedAgent.h"
#include "Leaf/L_Scale.h"
#include "Leaf/L_Shrink.h"
#include "Leaf/L_FollowAgent.h"
#include "Leaf/L_Jump.h"
