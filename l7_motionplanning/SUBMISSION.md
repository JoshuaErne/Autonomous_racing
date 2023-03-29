# Lab 7: RRT

## Video Link
[Simulation](https://youtu.be/mI7C_5F7xuQ) /
[F1TENTH Vehicle](https://youtu.be/1R6zaGeIvcw)

## Modifications/Extra Credit
Our version of RRT is planned in the local occupancy grid space. The occupancy grid is updated and the RRT tree is expanded on every scan callback. The major improvements we made are:
1. We directly use the pure pursuit waypoint as the ``goal`` for RRT tree. This avoids building unnecessary branches and allows our expansion to go directly to our goal point. If the pure pursuit waypoint lies on an obstacles then we resample from the free space until we find one that is close enough to the original goal.
2. We also implemented pruning on the path returned by RRT. The algorithm finds all possible collision-free sub-paths and calculate the cumulative distances needed to traverse all nodes in a path. It then returns path with the smallest distance. For example, if RRT returns path [a, b, c, d], our algorithm will test for collision and calculate the distances for paths [a, b, c, d], [a, d], [a, c, d], and [a, b, d] to find the most efficient one that is collision-free.

Our RRT algorithm works as follows:

1. Set ``start`` = car position, ``goal`` = pure pursuit waypoint
2. Check if ``goal`` is already occupied by an obstacle, resample near it if it is
3. Initialize 2 trees: ``T_start`` with root = start and ``T_goal`` with root = goal (or resampled goal point)
4. Sample a point from the free space of occupancy grid 
5. Find nearest node to the sampled point in ``T_start``
6. Check if traversing from the nearest node in ``T_start`` to the sampled point causes an collision
7. Add sampled point to the tree ``T_start`` if there is no collision and it brings the car closer to goal
8. Repeat steps 5, 6, and 7 for ``T_goal``
9. If a sampled point can be expanded to both ``T_start`` and ``T_goal`` (i.e it can form a path that connects ``start`` to ``goal``), then we stop and return this path
10. Repeat steps 4 - 9 until a path is found or a max iteration is reached. 
