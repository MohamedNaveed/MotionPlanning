# MotionPlanning

Implementation of the following algorithms:

### 1) A* search
![](https://github.com/MohamedNaveed/MotionPlanning/blob/main/a_star_plot.png)

#### Dijsktra (A* with heuristic = 0)
![](https://github.com/MohamedNaveed/MotionPlanning/blob/main/dijsktra_plot.png)

### 2) Breadth first search.
![](https://github.com/MohamedNaveed/MotionPlanning/blob/main/bfs_plot.png)

### 3) Rapidly Exploring Random Trees (RRT) 
![](https://github.com/MohamedNaveed/MotionPlanning/blob/main/RRT.png)

### 4) Rapidly Exploring Random Trees* (RRT*) 
![](https://github.com/MohamedNaveed/MotionPlanning/blob/main/RRTstar.png)

### 5) RRT* for vehicle lane change 
Search done in (x, y, theta) space with collision check implemented for rectangular obstacles.
![](https://github.com/MohamedNaveed/MotionPlanning/blob/main/lane_change_wRRTnodes.png)
![](https://github.com/MohamedNaveed/MotionPlanning/blob/main/lane_change_woRRTnodes.png)

### 6) RRT* for vehicle lane change using Kinematic Bicycle Model for steering. 
Search done in (x, y, theta) space with collision check implemented for rectangular obstacles. <br>
A proportional controller is used to generate control inputs for the model. 
![](https://github.com/MohamedNaveed/MotionPlanning/blob/main/RRTstarKBM_w_nodes.png)
![](https://github.com/MohamedNaveed/MotionPlanning/blob/main/RRTstarKBM_wo_nodes.png)

### 7) Trajectory using quintic polynomials for vehicle lane change ((x, y, theta) space) 
![](https://github.com/MohamedNaveed/MotionPlanning/blob/main/quintic_polynomial_path.png)
