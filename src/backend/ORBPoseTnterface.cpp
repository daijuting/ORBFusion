/*
 * This file is part of ORBFusion.
 */

#include "ORBPoseTnterface.h"


ORBPoseInterface::ORBPoseInterface()
{
    vTimestamps.clear();
    mPoses.clear();
    vTimestamps.reserve(3000);
    mPoses.reserve(3000);
    std::cout<<"ORBPoseInterface class initialized"<<std::endl;
}

void ORBPoseInterface::reset()
{
    vTimestamps.clear();
    mPoses.clear();
    std::cout<<"ORBPoseInterface class reset"<<std::endl;
}

void ORBPoseInterface::addCameraPose(uint64_t time,const Eigen::Matrix4f & pose)
{
    std::unique_lock<std::mutex> lock(ORBPosesMutex);
    vTimestamps.push_back(time);
    mPoses.push_back(pose);
}

bool ORBPoseInterface::getCameraPositions(std::vector<std::pair<uint64_t, Eigen::Vector3d> > & positions)
{
    std::unique_lock<std::mutex> lock(ORBPosesMutex);
    positions.clear();
    std::pair<uint64_t, Eigen::Vector3d> next;
    for(int i=0;i<vTimestamps.size();i++)
    {
        next.first = vTimestamps[i];
        next.second(0) = mPoses[i](0,3);
        next.second(1) = mPoses[i](1,3);
        next.second(2) = mPoses[i](2,3);
        positions.push_back(next);
    }
}

Eigen::Matrix4f ORBPoseInterface::getCameraPose(uint64_t time)
{
    std::unique_lock<std::mutex> lock(ORBPosesMutex);
    for(int i=0;i<vTimestamps.size();i++)
    {
        if(time == vTimestamps[i])
        {
            return mPoses[i];
        }
    }
    std::cout<<"ORBPoseInterface getCameraPose failed"<<std::endl;
    return Eigen::Matrix4f::Identity();
}

void ORBPoseInterface::optimise(std::vector<uint64_t> times,std::vector<Eigen::Matrix4f> poses)
{
    std::unique_lock<std::mutex> lock(ORBPosesMutex);
    assert(times.size()==poses.size());
    int N1 = times.size();
    int N2 = vTimestamps.size();
    for(int i=0;i<N1;i++)
    {
        for(int j=0;j<N2;j++)
        {
            if(vTimestamps[j] == times[i])
            {
                mPoses[j] = poses[i];
            }
        }
    }
}


