// DO NOT MODIFY 
// preparation.cpp controls this variable.

//savemode -> loadmode
#define SAVEMODE 0
#define LOADMODE 1


#define FOLDERNAME_const "5_10_복도5"
#define FILENUM_const 1728
#define PLYFILENAME "/home/cgvlab/ply_result_2022-1/5_10_복도5.ply"

#define NEURALMODE 0


/* HOW TO RUN the code

./preparation
catkin_make -DCMAKE_BUILD_TYPE=Release
roscore
roslaunch velodyne_pointcloud VLP16_points.launch
roslaunch zed_wrapper zed2.launch
roslaunch loam_velodyne loam_velodyne.launch

*/
