This repository includes code to compare the performance of two different approaches to graph-based visual odometry using RGB-D data. The approaches differ in their methods of obtaining initial guesses of the rigid body transformations between keyframes. An indirect front-end uses SURF to determine matched features between frames. These matches are used to create new keyframes and loop closures. A direct front-end instead uses Gauss-Newton, an iterative approach for computing the transformations. Both front-end methods send their initial guesses of the transformations relating each pair of connected keyframes to the same back-end method that jointly optimizes and updates each transformation using a pose synchronization method. The back-end also requires the loop closures between keyframes, which are only determined through our indirect front-end.

# How to run a simple test:

## 1. Path setting
- Set your path at ./RGB-D_DSO
- Add all the subfolders in RGB-D_DSO to your path.

## 2. Indirect Method
- Run ./indirect/indirect_main.m
  - This will implement the indirect front-end on a small 30-frame sample of the TUM Freiburg2 dataset (freiburg2.mat), and jointly optimize using the back-end joint_optimization.m
  - This will also output a keys.mat file that the direct front-end will read when it is run to determine which loop closures were found by the indirect front-end.
  - This will also plot some results of the indirect front-end method.

## 3. Direct Method
- Run ./direct/direct_main.m
  - This will use the loop closure info from the indirect front-end and generate alignment results of the direct front-end (using the same back-end joint_optimization.m)
  - You can visualize the process of alignment by uncomment line 231-233 in ./direct/@rgbd_dvo/rgbd_dvo.m

# How to evaluate your result with ground truth:
- After run through direct_main.m or indirect_main.m, store your coordinate variable as a mat file. It will be used to generate a trajectory file in the next step.
- Run ./gen_traj_txt.m and enter the name of the txt file in line 17.
- To generate a RPE (relative pose error) plot, run ./compare_traj/evaluate_rpe.py freiburg2_gt.txt (your_traj_txt_name).txt --fixed_delta --plot (Plot_you_want_to_name_as)

# References
- Steinbrücker, F., Sturm, J., & Cremers, D. (2011, November). Real-time visual dometry from dense RGB-D images. In 2011 IEEE International Conference on Computer Vision Workshops (ICCV Workshops) (pp. 719-722). IEEE.
- Engel, J., Koltun, V., \& Cremers, D. (2018). Direct sparse odometry. IEEE transactions on pattern analysis and machine intelligence, 40(3), 611-625.
- Computer Vision Group,
TUM Department of Informatics. Useful tools for the RGB-D benchmark. Retrieved from https://vision.in.tum.de/data/datasets/rgbd-dataset/tools.
