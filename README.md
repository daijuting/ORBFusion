# ORBFusion #

This repository is for 'ORBFusion: Real-time and Accurate dense SLAM at large scale', which is capable of producing high quality globally consistent point and mesh reconstructions in real-time at large scale with RGB-D images.

# Related Publications #
If ORBFusion is helpful for your research, please consider citing :

* **[ORBFusion: Real-time and Accurate dense SLAM at large scale]**, *J. Dai, X. Tang, L. Oppermann*, IEEE ISMAR 2017

# 1:Prerequisites #
* Ubuntu 14.04, 16.04 (We have tested the library in `Ubuntu 14.04` and `16.04`, but it should be easy to compile in other platforms)
* CMake
* OpenGL
* [CUDA >= 7.0](https://developer.nvidia.com/cuda-downloads)
* [OpenNI2](https://github.com/occipital/OpenNI2)
* SuiteSparse
* Eigen
* Boost
* zlib
* libjpeg
* [OpenCV](http://sourceforge.net/projects/opencvlibrary/files/opencv-unix/2.4.9/opencv-2.4.9.zip)
* [DLib](https://github.com/dorian3d/DLib) @ 330bdc10576f6bcb55e0bd85cd5296f39ba8811a
* [DBoW2](https://github.com/dorian3d/DBoW2) @ 4a6eed2b3ae35ed6837c8ba226b55b30faaf419d
* [DLoopDetector](https://github.com/dorian3d/DLoopDetector) @ 84bfc56320371bed97cab8aad3aa9561ca931d3f
* [iSAM](http://people.csail.mit.edu/kaess/isam/)
* [PCL](http://pointclouds.org/)
* [Pangolin](https://github.com/stevenlovegrove/Pangolin)
* C++11 or C++0x Compiler

# 2: Test #

### TUM Dataset

1. Download a sequence from http://vision.in.tum.de/data/datasets/rgbd-dataset/download and create the corresponding `associations.txt` file, and put the file under the dataset directory.

2. We provide a script `build.sh` to build the Thirdparty libraries needed and ORBFusion. Please make sure you have installed all required dependencies (section 1) and change the corresponding dataset address to your own in file `ORBFusion/src/MainControlle.cpp, Then execute:
```
cd ORBFusion
chmod +x build.sh
./build.sh
```

This will create a build folder and executable `ORBFusion` in build folder, executing the following command.
```
cd build
./ORBFusion
```

