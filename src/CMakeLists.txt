cmake_minimum_required(VERSION 2.6.0)

project(ORBFusion)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(OpenNI2 REQUIRED)
find_package(Pangolin 0.1 REQUIRED)
find_package(OpenCV 2.4 REQUIRED)
find_package(ZLIB REQUIRED)
find_package(CUDA 7.0 REQUIRED)
find_package(DLib REQUIRED)
find_package(DBoW2 REQUIRED)
find_package(DLoopDetector REQUIRED)
find_package(SuiteSparse REQUIRED)
find_package(iSAM REQUIRED)
find_package(Boost COMPONENTS thread serialization REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${ZLIB_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIR})
include_directories(${DLib_INCLUDE_DIRS})
include_directories(${DBoW2_INCLUDE_DIRS})
include_directories(${OPENNI2_INCLUDE_DIR})
include_directories(${DLoopDetector_INCLUDE_DIRS})
include_directories(${SUITESPARSE_INCLUDE_DIRS})
include_directories(${ISAM_INCLUDE_DIRS})

file(GLOB srcs *.cpp *.h*)
file(GLOB utils utils/*.cpp utils/*.h*)
file(GLOB frontend frontend/*.cpp frontend/*.h*)
file(GLOB cuda frontend/cuda/*.cu)
file(GLOB containers frontend/cuda/containers/*.cpp)
file(GLOB backend backend/*.cpp backend/*.h*)
file(GLOB pnp backend/pnp/*.cpp backend/pnp/*.h*)

#for orb-slam module
file(GLOB orb SlamEngine/src/*.cpp SlamEngine/include/*.h)

file(GLOB DBOW2 SlamEngine/Thirdparty/DBoW2/DBoW2/*.cpp SlamEngine/Thirdparty/DBoW2/DBoW2/*.h
                SlamEngine/Thirdparty/DBoW2/DUtils/*.cpp SlamEngine/Thirdparty/DBoW2/DUtils/*.h)
file(GLOB g2o SlamEngine/Thirdparty/g2o/g2o/core/*.h*  SlamEngine/Thirdparty/g2o/g2o/core/*.cpp
              SlamEngine/Thirdparty/g2o/g2o/solvers/*.h*
              SlamEngine/Thirdparty/g2o/g2o/stuff/*.h*  SlamEngine/Thirdparty/g2o/g2o/stuff/*.cpp
              SlamEngine/Thirdparty/g2o/g2o/types/*.h*  SlamEngine/Thirdparty/g2o/g2o/types/*.cpp
    )

message("PROJECT_SOURCE_DIR points to " ${PROJECT_SOURCE_DIR})
set(CUDA_ARCH_BIN "20 30 35 50 52" CACHE STRING "Specify 'real' GPU arch to build binaries for, BIN(PTX) format is supported. Example: 1.3 2.1(1.3) or 13 21(13)")
set(CUDA_ARCH_PTX "" CACHE STRING "Specify 'virtual' PTX arch to build PTX intermediate code for. Example: 1.0 1.2 or 10 12")              

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR})
include(CudaComputeTargetFlags.cmake)                  
APPEND_TARGET_ARCH_FLAGS()

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}  "-Xcompiler;-fPIC;")           
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "--ftz=true;--prec-div=false;--prec-sqrt=false") #for my laptop
#set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "--ftz=true;--prec-div=false;--prec-sqrt=false;-std=c++11") #for server PC

CUDA_COMPILE(cuda_objs ${cuda})   

#Include only after compiling with nvcc, this stops vtk polluting nvcc's definitions with whitespace
find_package(PCL 1.7 QUIET REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -msse -msse2 -msse3 -O3 -std=c++11")
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g -std=c++11")

#LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/SlamEngine/cmake_modules)
find_package(Eigen3 3 REQUIRED)
link_directories(${EIGEN3_INCLUDE_DIR})
add_library(SlamEngine STATIC
    ${orb}
    ${DBOW2}
    ${g2o}
#    ${PROJECT_SOURCE_DIR}/SlamEngine/Thirdparty/DBoW2/lib/libDBoW2.so
#    ${PROJECT_SOURCE_DIR}/SlamEngine/Thirdparty/g2o/lib/libg2o.so
    )

add_library(util STATIC ${utils})
add_library(frontend STATIC ${frontend} ${cuda} ${cuda_objs} ${containers})
add_library(backend STATIC ${backend} ${pnp})

#add below for file under /SlamEngine/src /SLamEngine/include to find Caffe library
#target_link_libraries(SlamEngine
#    ${OpenCV_LIBS}
#    ${EIGEN3_LIBS}
#    ${Pangolin_LIBRARIES}
#    ${PROJECT_SOURCE_DIR}/SlamEngine/Thirdparty/DBoW2/lib/libDBoW2.so
#    ${PROJECT_SOURCE_DIR}/SlamEngine/Thirdparty/g2o/lib/libg2o.so
#    ${PCL_LIBRARIES}

#    #${Caffe_LIBRARIES}
#    )

add_executable(ORBFusion
               ${srcs}
               )

target_link_libraries(ORBFusion
                      util
                      frontend
                      backend
                      SlamEngine
                      ${PCL_LIBRARIES}
                      ${ZLIB_LIBRARY}
                      ${Pangolin_LIBRARIES}
                      ${Boost_LIBRARIES} 
                      ${CUDA_LIBRARIES}
                      ${OpenCV_LIBS}
                      ${DLib_LIBS} 
                      ${DBoW2_LIBS}
                      ${OPENNI2_LIBRARY}
                      ${SUITESPARSE_LIBRARIES}
                      ${ISAM_LIBRARIES}
                      #for orb-slam
                      ${EIGEN3_LIBS}
                      )

