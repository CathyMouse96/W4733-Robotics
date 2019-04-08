## Assumptions

We are assuming that the robot is a 36 x 36 square and the reference point is the square center.

## Usage

### Task 1: Grow obstacles.

`python src/grow_obstacles.py`

By default, this reads data from `data/world_obstacles.txt` and `data/goal.txt`, grows the obstacles, and plots the obstacles (before and after) with `matplotlib`. You can see the result at `results/Q1.png`.

### Task 2: Create visibility graph.

`python src/gene_vgraph.py`

This generates the visibility graph and plots the graph with `matplotlib`. You can see the result at `results/Q2.png`.

### Task 3: Compute shortest path.

`python src/shortest_path.py`

This computes the shortest path in the visibility graph with Dijkstra's algorithm and plots the path with `matplotlib`. You can see the result at `results/Q3.png`.

### Task 4: Move the robot.

First ensure that you have the environment set up and ROS running.

`python markers.py`

This visualizes the results from tasks 1, 2 and 3 in RViz.

`python move_robot.py`

This moves the robot along the shortest path. Way to go!

## Implementation

Our implementation is fairly straightforward. One design choice worthing noting is for removing edges that collide with obstacles. We wrote our own collision checker based on a trick we found on the Internet. We had to work out the details of what to do when an edge shared an endpoint with the edge of an obstacle.

While applying code from `odom_out_and_back.py` was mostly sufficient, we had to modify `goal_angle`. In `odom_out_and_back.py`, when the robot started to move, its rotation was always zero. But this is not true in our case, because the robot is moving between several points. So we had to modify `goal_angle` to `goal_angle - rotation`.

As we can see, the robot mainly heads directly to the goal, but sways left at first due to the first obstacle.
