This repository includes code to compare the performance of two different approaches to graph-based visual odometry using RGB-D data. The approaches differ in their methods of obtaining initial guesses of the rigid body transformations between keyframes. An indirect front-end uses SURF to determine matched features between frames. These matches are used to create new keyframes and loop closures. A direct front-end instead uses Gauss-Newton, an iterative approach for computing the transformations. Both front-end methods send their initial guesses of the transformations relating each pair of connected keyframes to the same back-end method that jointly optimizes and updates each transformation using a pose synchronization method. The back-end also requires the loop closures between keyframes, which are only determined through our indirect front-end.

How to run a simple test:
- Navigate to the test/ folder
- Run indirect_main.m
  - This will implement the indirect front-end on a small 30-frame sample of the TUM Freiburg2 dataset (freiburg2.mat), and jointly optimize using the back-end joint_optimization.m
  - This will also output a keys.mat file that the direct front-end will read when it is run to determine which loop closures were found by the indirect front-end.
  - This will also plot some results of the indirect front-end method.
- Run direct_main.m
  - This will use the loop closure info from the indirect front-end and plot results of the direct front-end (using the same back-end joint_optimization.m)
