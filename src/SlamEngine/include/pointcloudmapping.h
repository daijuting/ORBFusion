/*
 * This file is part of ORBFusion.
 */

#ifndef _POINTCLOUDMAPPING_H
#define _POINTCLOUDMAPPING_H

#include "System.h"

#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>

#include <opencv2/highgui/highgui.hpp>

#include <condition_variable>
#include <thread>
#include <mutex>

#include "KeyFrame.h"
#include "Converter.h"

#include <iostream>
#include <vector>

using namespace std;

namespace ORB_SLAM2
{

class PointCloudMapping
{
public:
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
    
    PointCloudMapping( double resolution_ );
    
    // 插入一个keyframe，会更新一次地图
    void insertKeyFrame( KeyFrame* kf, cv::Mat& color, cv::Mat& depth );

    void viewer();
    void savePointCloudMap(const string &filename);

    void RequestFinish();
    bool CheckFinish();
    void SetFinish();
    bool isFinished();
    
protected:
    PointCloud::Ptr generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth);
    
    PointCloud::Ptr globalMap;
    
    bool    mbFinishRequested = false;
    bool    mbFinished = false;
    std::mutex   mMutexFinish;
    
    condition_variable  keyFrameUpdated;
    std::mutex  keyFrameUpdateMutex;
    

    std::mutex  keyframeMutex;
    uint16_t    lastKeyframeSize =0;
    
    double resolution = 0.04;
    pcl::VoxelGrid<PointT>  voxel;

    // data to generate point clouds
    std::vector<KeyFrame*>  keyframes;

    std::vector<cv::Mat>    colorImgs;
    std::vector<cv::Mat>    depthImgs;
};

} //namespace ORB_SLAM

#endif // POINTCLOUDMAPPING_H
