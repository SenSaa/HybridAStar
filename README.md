# HybridAStar
Hybrid A Star in Unity (C#)


To simplify Hybrid A*, it is essentially A*, embedded with a vehicle model for node expansions, and Reeds Shepp path for when the path to the goal is obstacle free.
Further optimisations such as voronoi fields and conjugate gradient smoothing are not used here.

This algorithm is useful for unstructured planning for autonomous vehicles and mobile robotics.

I added a small modification by applying weight to the A* heuristic (Weighted A*), making it faster at the cost of sub-optimality.

The user has access to the start position, goal position, start yaw, and goal yaw.

https://user-images.githubusercontent.com/19212519/206953127-2c7bdd70-1f9d-4846-b314-28c5ac0cefa5.mov

Based on:

Python Implementation: https://github.com/jhan15/dubins_path_planning

Paper: Practical Search Techniques in Path Planning for Autonomous Driving

https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf
