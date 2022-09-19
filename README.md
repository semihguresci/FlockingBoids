# FlockingBoids Simulation implemented in CPP

The provided framework is a simple application which simulates basic flocking behaviour of a bunch of entities (boids).
The simulation is written in an extremely basic and naive way. 

the game manager initializes the boid manager, and triggers the simulation steps for the system every frame (_boidMng->stepSimulation();), as well as the settings of the render data for the boids (_boidMng->syncWindowStep();).
The simulation and render data setting stages are separate, due to the rendering code running on a different thread than the game code.

these defines in sg_gameMng.cpp determine the number of boids:
#define NUM_SMALL_BOIDS 900
#define NUM_BIG_BOIDS 4

