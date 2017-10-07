/*
 * This file is part of ORBFusion.
 */

#ifndef ORBPOSEINTERFACE_H
#define ORBPOSEINTERFACE_H


#include<iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <map>
#include <Eigen/Dense>
#include <mutex>
#include <vector>
#include <stdint.h>

class ORBPoseInterface
{
public:
    ORBPoseInterface();
    void reset();
    void addCameraPose(uint64_t time1,const Eigen::Matrix4f & tprev);

    bool getCameraPositions(std::vector<std::pair<uint64_t, Eigen::Vector3d> > & positions);

    void optimise(std::vector<uint64_t> vTimestamps,std::vector<Eigen::Matrix4f> mPoses);

    Eigen::Matrix4f getCameraPose(uint64_t id);
private:
//    vector<Eigen::Matrix4f> prev = Eigen::Matrix4f::Identity();

    std::vector<uint64_t> vTimestamps;
    std::vector<Eigen::Matrix4f> mPoses;
    std::mutex  ORBPosesMutex;
};

#endif // ORBPOSEINTERFACE_H
