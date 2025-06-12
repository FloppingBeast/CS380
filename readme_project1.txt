Student Name: Lawrence Winters (lawrence.winters@digipen.edu)

Project Name: BackyardSoccer

What I implemented:
- I implemented a variety of behavior nodes such as clockwise and counterclockwise rotations, scaling and shrinking, following a randomly selected agent, and jumping. I implemented a new control flow node called FlipFlop which will save the last the index of the child it was on and continue to flip flop between the child nodes, always picking up from the last ran one if a failure occurs. I also implemented a proximity checking decorator which allows an agent to perform a behavior based on if there is something within its' proximity. Lastly, I implemented a skip first time decorator which will skip the first run of behavior.

Directions (if needed):
- I left the ability to left-click the mouse button and tied some behavior to that so you can see a few agents move or react differently to the mouse click. 

What I liked about the project and framework:
- The framework was fairly easy to pick up and the work flow wasn't too bad either. I didn't need look at the documentation on the slides too much as I thought the project was very well organized and easy to follow. I also liked the behavior tree editor, as it made it easy to see how many nodes were in a tree and what the depth of a tree was. Lastly, I thought the addition of the sound into the engine was a nice touch, even thought I only found one sound file for my tree.

What I disliked about the project and framework:
- I really disliked the use of the blackbox in the project. There were times when I would want to see if a value had been saved to the black box but this not possible, as if a value is not found the project breaks. I also didn't like the .sdkmesh use for assets because I couldn't find any sdkmeshes on the internet and the fbx converter I used cause a corruption in a model of cat that I wanted to use. I also thought the instructions were a bit vague in the sense of having 4 NPCs doing something interesting. I wasn't too clear on what constituted interesting so I had my guys try to be actively moving, spinning, scaling and jumping to ensure something was constantly happening on screen at the very least.

Any difficulties I experienced while doing the project: 
- I had a bit of trouble dealing with a work around for the black box and saving an agent that was in proximity or needed for following. In the end I decided to just save a nullptr and send the behavior node to failure if this was used without the proper saving of an agent. I also struggled a bit with making sure a decorator was acting as a decorator and not as a behavior node, which was interesting because I think the proximity checker could've acted as both and decorator or a behavior. The only other thing I got 

Hours spent: ~12 hours

New selector node (name): C_FlipFlop

New decorator nodes (names): D_ProximityChecker, D_SkipFirstTime, D_SelectRandomAgent

10 total nodes (names): C_FlipFlop, D_ProximityChecker, D_SkipFirstTime, D_SelectRandomAgent, L_SpinClockwise, L_SpinCounterClockwise, L_MoveToSavedAgent, L_Scale, L_Shrink, L_FollowAgent, L_Jump

4 Behavior trees (names): Ball.bht, Bird.bht, Man.bht, Tree.bht