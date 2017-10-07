/*
 * This file is part of ORBFusion,modified from Kintinuous.
 */

#include "GroundTruthOdometry.h"

GroundTruthOdometry::GroundTruthOdometry(std::vector<Eigen::Vector3f> & tvecs_,
                                         std::vector<Eigen::Matrix<float, 3, 3, Eigen::RowMajor> > & rmats_,
                                         std::map<uint64_t, Eigen::Isometry3f, std::less<int>, Eigen::aligned_allocator<std::pair<const uint64_t, Eigen::Isometry3f> > > & camera_trajectory,
                                         uint64_t & last_utime)
 : tvecs_(tvecs_),
   rmats_(rmats_),
   camera_trajectory(camera_trajectory),
   last_utime(last_utime)
{

}

GroundTruthOdometry::~GroundTruthOdometry()
{

}

void GroundTruthOdometry::reset()
{
    return;
}

CloudSlice::Odometry GroundTruthOdometry::getIncrementalTransformation(Eigen::Vector3f & trans,
                                                                       Eigen::Matrix<float, 3, 3, Eigen::RowMajor> & rot,
                                                                       const DeviceArray2D<unsigned short> & depth,
                                                                       const DeviceArray2D<PixelRGB> & image,
                                                                       uint64_t timestamp,
                                                                       unsigned char * rgbImage,
                                                                       unsigned short * depthData)
{
    if (last_utime != 0 && !camera_trajectory.empty())
    {

#if 1
        Eigen::Isometry3f camera_pose;

        if(!camera_trajectory.empty())
        {
            std::map<uint64_t, Eigen::Isometry3f>::const_iterator it = camera_trajectory.find(last_utime);
            if (it == camera_trajectory.end())
            {
                return CloudSlice::GROUNDTRUTH;
            }
            camera_pose = it->second;
        }
        else
        {
            return CloudSlice::GROUNDTRUTH;
        }

        trans = camera_pose.translation();
        rot = camera_pose.rotation();
#else
        Eigen::Matrix<float, 3, 3, Eigen::RowMajor> Rprev = rmats_[rmats_.size() - 1];
        Eigen::Vector3f tprev = tvecs_[tvecs_.size() - 1];
        Eigen::Isometry3f delta = camera_trajectory[last_utime].inverse() * camera_trajectory[timestamp];
        Eigen::Isometry3f currentTsdf;
        currentTsdf.setIdentity();
        currentTsdf.rotate(Rprev);
        currentTsdf.translation() = tprev;

        Eigen::Matrix4f M;
        M <<  0,  0, 1, 0,
             -1,  0, 0, 0,
              0, -1, 0, 0,
              0,  0, 0, 1;

        currentTsdf = currentTsdf * M.inverse() * delta * M;

        trans = currentTsdf.translation();
        rot = currentTsdf.rotation();
#endif
    }

    return CloudSlice::GROUNDTRUTH;
}

Eigen::MatrixXd GroundTruthOdometry::getCovariance()
{
    Eigen::MatrixXd cov(6, 6);
    cov.setIdentity();
    cov(0, 0) = 0.1;
    cov(1, 1) = 0.1;
    cov(2, 2) = 0.1;
    cov(3, 3) = 0.5;
    cov(4, 4) = 0.5;
    cov(5, 5) = 0.5;
    return cov;
}

bool GroundTruthOdometry::preRun(unsigned char * rgbImage,
                                 unsigned short * depthData,
                                 uint64_t timestamp)
{
    Eigen::Isometry3f camera_pose;

    if(!camera_trajectory.empty())
    {
        std::map<uint64_t, Eigen::Isometry3f>::const_iterator it = camera_trajectory.find(timestamp);
        if (it == camera_trajectory.end())
        {
            return false;
        }
        camera_pose = it->second;
    }
    else
    {
        return false;
    }

    return true;
}
