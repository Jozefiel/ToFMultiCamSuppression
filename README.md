
# ToFMultiCamSuppression
This is repository for ToF multi-camera interference suppression.
When ToF cameras like Microsoft Kinect v2 work in parallel mode, the interference in IR and D frames is occurred. This interference can be suppressed using this algorithm.

For build program must be installed

 - PCL 1.9.1 (another version can be set in CMakeLists.txt) 
 - OpenCV  
 - VTK
 - Threads

Building in terminal:

    git clone https://github.com/Jozefiel/ToFMultiCamSuppression.git

    cd ToFMultiCamSuppression

    mkdir build
    cd build
    cmake ..
    make -j 2
    cp Interference ../hlava1/data

for test on Dataset hlava1

    cd ../hlava1/data
    ./run_interference_program.sh main

This script consist of:
 - collectData : create buffers in directories
 - filtration  : run Interference program in buffer directories
 - computeHausdorff: compute hausdorff distance between ref model and filtered model 
	 - must be installed pcl_ply2pcd and pcl_compute_hausdorff binaries
 - resultCollecting : collect results from hausdorff distance and from time computing

Each part can be run separately, where is need to select number for dataset (0,1,2)
example:

    ./run_interference_program.sh collectData 0
    ./run_interference_program.sh collectData 1
    ./run_interference_program.sh resultCollecting 1


