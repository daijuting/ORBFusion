/*
 * This file is part of ORBFusion.
 */

#include "../include/pointcloudmapping.h"

namespace ORB_SLAM2
{

PointCloudMapping::PointCloudMapping(double resolution_)
{
    this->resolution = resolution_;
    voxel.setLeafSize( resolution, resolution, resolution);
    globalMap = PointCloud::Ptr(new PointCloud());

}

void PointCloudMapping::RequestFinish()
{
    unique_lock<mutex> lck(mMutexFinish);
    mbFinishRequested = true;
    keyFrameUpdated.notify_one();
}

bool PointCloudMapping::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void PointCloudMapping::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool PointCloudMapping::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void PointCloudMapping::insertKeyFrame(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
{
    cout << "receive a keyframe, id = " << kf->mnId << endl;
    unique_lock<mutex> lck(keyframeMutex);
    keyframes.push_back( kf );
    colorImgs.push_back( color.clone() );
    depthImgs.push_back( depth.clone() );
    
    keyFrameUpdated.notify_one();
}

pcl::PointCloud< PointCloudMapping::PointT >::Ptr PointCloudMapping::generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
{
    PointCloud::Ptr tmp( new PointCloud() );
    // point cloud is null ptr
    for ( int m=0; m<depth.rows; m+=3 )
    {
        for ( int n=0; n<depth.cols; n+=3 )
        {
            float d = depth.ptr<float>(m)[n];//the depth is already scaled
            if (d < 0.01 || d>10)
                continue;
            PointT p;
            p.z = d;
            p.x = ( n - kf->cx) * p.z / kf->fx;
            p.y = ( m - kf->cy) * p.z / kf->fy;
            
            p.b = color.ptr<uchar>(m)[n*3];
            p.g = color.ptr<uchar>(m)[n*3+1];
            p.r = color.ptr<uchar>(m)[n*3+2];
                
            tmp->points.push_back(p);
        }
    }
    
    Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat( kf->GetPose() );
    PointCloud::Ptr cloud(new PointCloud);
    pcl::transformPointCloud( *tmp, *cloud, T.inverse().matrix());
    cloud->is_dense = false;
    
    cout<<"generate point cloud for kf "<<kf->mnId<<", size="<<cloud->points.size()<<endl;
    return cloud;
}


void PointCloudMapping::viewer()
{
    pcl::visualization::CloudViewer viewer("viewer");
    while(1)
    {
        {
            unique_lock<mutex> lck_keyframeUpdated( keyFrameUpdateMutex );
            keyFrameUpdated.wait( lck_keyframeUpdated );
        }
        {
            if (CheckFinish())
            {
                savePointCloudMap("pointCloudMap.pcd");
                {
                    unique_lock<mutex> lock(mMutexFinish);//finished also have unique_lock<mutex> lock(mMutexFinish) in it
                    SetFinish();
                }

                break;
            }
        }
        
        // keyframe is updated 
        size_t N=0;
        {
            unique_lock<mutex> lck( keyframeMutex );
            N = keyframes.size();
        }
        //test function rum time
        double duration = static_cast<double>(cv::getTickCount());

        {
            unique_lock<mutex> lck( keyframeMutex );
            for ( size_t i=lastKeyframeSize; i<N ; i++ )
            {
                PointCloud::Ptr p = generatePointCloud( keyframes[i], colorImgs[i], depthImgs[i] );
                *globalMap += *p;
            }
        }

//        PointCloud::Ptr tmp(new PointCloud());
//        voxel.setInputCloud( globalMap );
//        voxel.filter( *tmp );
//        globalMap->swap( *tmp );
//        viewer.showCloud( globalMap );
        cout<<"show global map, size="<<globalMap->points.size()<<endl;
        lastKeyframeSize = N;

        //test function rum time
        duration = static_cast<double>(cv::getTickCount())-duration;
        duration /= (double)cv::getTickFrequency();//unit "s"
        cout<<"KeyFrameNum:"<<lastKeyframeSize<<" rum time:"<<duration<<" S"<<endl;
    }
}

void PointCloudMapping::savePointCloudMap(const string &filename)
{
    pcl::io::savePCDFile(filename.c_str(), *globalMap);
    globalMap->points.clear();
    cout<<"PointCloudMap saved."<<endl;
}

} //namespace ORB_SLAM
