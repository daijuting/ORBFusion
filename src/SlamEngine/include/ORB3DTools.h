/*
 * This file is part of ORBFusion,modified from Kintinuous.
 */

#ifndef ORB3DTOOLS_H
#define ORB3DTOOLS_H


#include<iostream>
#include <opencv2/opencv.hpp>

#include <condition_variable>
#include <thread>
#include <mutex>
#include <vector>

#include <Eigen/Dense>

#include "KeyFrame.h"
#include "Converter.h"

#include "../../backend/PlaceRecognition.h"
#include "../../backend/Deformation.h"

#include "../include/ORBmatcher.h"



namespace ORB_SLAM2
{

class ORB3DTools
{
public:
    ORB3DTools(Deformation *_mDeformation);
    ORB3DTools();

    void ORBmatches(KeyFrame* mpQuery,KeyFrame* mpMatched);

    void myORBFeaturesTest(cv::Mat img_1,cv::Mat img_2,KeyFrame* mpQuery,KeyFrame* mpMatched);

    cv::Point3f point2dTo3d( cv::Point3f& point,const float fx,const float fy,const float cx,const float cy);

    int estimateMotion( std::vector<cv::DMatch>  &goodMatches,const cv::Mat mk,
                        const std::vector<cv::KeyPoint> mvKeysquery,const std::vector<cv::KeyPoint> mvKeysmatched,
                        const std::vector<float> mvDepthQuery,const std::vector<float> mvDepthMatched);

    void getMatches(cv::BFMatcher matcher,const cv::Mat& queryDesc,
                                const cv::Mat & matchedDesc,std::vector<cv::DMatch>& matches);

    bool refineMatchesWithHomography(   const std::vector<cv::KeyPoint>& queryKeypoints,
                                        const std::vector<cv::KeyPoint>& trainKeypoints,
                                        float reprojectionThreshold,
                                        std::vector<cv::DMatch>& matches,
                                        cv::Mat& homography
                                    );
    Eigen::Vector3d point2dToEigen3d( cv::Point3f& point,
                                      const float fx,const float fy,const float cx,const float cy);

    void projectInlierMatches(std::vector<cv::DMatch>& goodMatches,
                                           std::vector<Eigen::Vector3d> & inlQurey,
                                           std::vector<Eigen::Vector3d> & inlMatched,
                                           KeyFrame* mpQuery,
                                           KeyFrame* mpMatched);

    void ByMinDistance(std::vector<cv::DMatch> &matches, const double threshold);

    Eigen::Matrix4f convertToEigen4f(KeyFrame* mKeyframe);

    void timeTest(KeyFrame* mpQuery,KeyFrame* mpMatched);

//private:
    Deformation *mpDeformation;
};

} //namespace ORB_SLAM


#endif // ORB3DTOOLS_H
