# PathPlanningSystem

## Abstract
In this work, an implementation of the probabilistic road maps algorithm presented by *Faverjon, Bernard and Pierre Tournassoud.* has been carried out. It is based on the work previously done in *Sakai, Atsushi et al*. From this work the architecture of the implementation and its implementation of the dijkstra algorithm for optimal path finding has been taken. 

## Installation requirements
This code only needs a python 3 installation and the libraries indicated in the header to work.
Required libraries:
* [numpy](https://numpy.org/) presented in *Harris, Charles R. et al*.
* [matplotlib.pyplot](https://numpy.org/) presented in *Hunter, John D*.
* [scipy.spatial](https://docs.scipy.org/doc/scipy/index.html) *Virtanen, Pauli et al*.

The main method calls a method that returns the obstacle points of the maze. There are three diferent methods with different complexity. By default generate_maze_easy() is executed which generates the simplest maze. To generate mazes of other difficulty you can comment out the line of code that calls this method and uncomment the lines that call the other methods

## Analysis 

There are three different mazes with three levels of difficulty. The difficulty varies according to the number of obstacles and the space available for the movement of the robot. The more obstacles there are, the less movement, which means a more difficult maze. The problem is posed as a robot of size r that has to navigate from an initial point to an end point. These points have to be valid i.e. the robot must be able to fit into the point without coming into contact with any obstacle. The three levels of difficulty have been proposed to be solved with robots of size r = 3 and r = 5. The following are the tests that have been performed on the different levels and configurations for the robot.

* Mode: EASY. Robot_size=3
<p align="center">
  <img src="/pictures/o10_r3_easy.png" width=448px height=336px title="Mode: EASY. Robot_size = 3">
</p>

* Mode: EASY. Robot_size=5
<p align="center">
  <img src="/pictures/o10_r5_easy.png" width=448px height=336px title="Mode: EASY. Robot_size = 5">
</p>

* Mode: EASY. Robot_size=5
<p align="center">
  <img src="/pictures/o10_r5_easy_fail.png" width=448px height=336px title="Mode: EASY. Robot_size = 5">
</p>

As the image shows, the robot does not always overcome the problem if the robot has a size r = 5. This is because the random points that are generated have very little space available near the obstacles, so these points are distributed over the free space leaving the obstacle spaces sparsely populated. This can lead to occasions where there are isolated areas and unconnected points in the network, so that the algorithm cannot generate a road map and complete the execution. This behaviour is showned also at the following levels:


* Mode: MEDIUM. Robot_size=3
<p align="center">
  <img src="/pictures/o10_r3_medium.png" width=448px height=336px title="Mode: MEDIUM. Robot_size=3">
</p>

* Mode: MEDIUM. Robot_size=5
<p align="center">
  <img src="/pictures/o10_r5_medium_fail.png" width=448px height=336px title="Mode: MEDIUM. Robot_size=5">
</p>

* Mode: HARD. Robot_size=3
<p align="center">
  <img src="/pictures/o10_r3_hard.png" width=448px height=336px title="Mode: HARD. Robot_size=3">
</p>

* Mode: HARD. Robot_size=5
<p align="center">
  <img src="/pictures/o10_r5_hard_fail.png" width=448px height=336px title="Mode: HARD. Robot_size=5">
</p>

# Implementation details

The probabilistic road map algorithm consists of three general points:

*Generate a series of random points on the map.

*Connect those points together generating a road map.

*Calculate the best possible route within the road map using a search algorithm such as Dijkstra or A*.

Unlike the *Sakai, Atsushi et al* work that uses a kd-tree to find nearest neighbors in the road map creation phase, this implementation has several methods that calculate and order the distances of each node to its neighbors. No differences in execution times have been observed between the two implementations. This may be because the dimensions of the map and therefore the number of random points generated is low. Since the time complexity of the method to calculate the distances is O(n^2), since for each point the distance to the rest of the points of the map is calculated, while the in-memory complexity is O(n). This could be optimized in multiple ways, one of them could be to calculate the distance of a number n of neighbors, enough so that the hope of selecting the maximum number of neighbors per node has a sufficiently high value (this could be calculated by statistical analysis). This maximum value is posed at the beginning of the code and is used to define the maximum number of connections that each node will have in the road map. Once the distance of all the neighbors with respect to the node is known, the distances are ordered with the [sorted](https://docs.python.org/3/library/functions.html#sorted) method, which has a complexity O(n log n). On the other hand, the kd-tree has a complexity of O(n log n), so it is more advisable to use this option. In this implementation it has not been used in order to differ from the one proposed in the reference work.

Finally, it is worth mentioning that there are better options than Dijkstra's algorithm for optimal path finding, such as the A* algorithm, among others. 

##Further work

* Reducing the implementation complexity
* Implementing a more efficient search algorithm

## References
Sakai, Atsushi et al. “PythonRobotics: a Python code collection of robotics algorithms.” ArXiv abs/1808.10703 (2018): n. pag.

Harris, Charles R. et al. “Array programming with NumPy.” Nature 585 (2020): 357 - 362.

Hunter, John D.. “Matplotlib: A 2D Graphics Environment.” Computing in Science & Engineering 9 (2007): n. pag.

Virtanen, Pauli et al. “SciPy 1.0: fundamental algorithms for scientific computing in Python.” Nature Methods 17 (2020): 261 - 272.

Faverjon, Bernard and Pierre Tournassoud. “Probabilistic Roadmaps for Path Planning in High-Dimensional Configuration Spaces.” (1996).
