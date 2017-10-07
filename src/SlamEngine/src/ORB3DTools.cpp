/*
 * This file is part of ORBFusion.
 */

#include "../include/ORB3DTools.h"


namespace ORB_SLAM2
{

ORB3DTools::ORB3DTools(Deformation *_mDeformation):mpDeformation(_mDeformation)
{
}
ORB3DTools::ORB3DTools()
{
}

void ORB3DTools::ByMinDistance(std::vector<cv::DMatch> &matches, const double threshold)
{
    vector< cv::DMatch > matches2;
    double minDis = 99999;
    for ( size_t i=0; i<matches.size(); i++ )
    {
        if ( matches[i].distance < minDis )
            minDis = matches[i].distance;
    }

    for ( size_t i=0; i<matches.size(); i++ )
    {
        if (matches[i].distance <= minDis*threshold)
            matches2.push_back( matches[i] );
    }
    matches.swap(matches2);
}

void ORB3DTools::myORBFeaturesTest(cv::Mat img_1,cv::Mat img_2,KeyFrame* mpQuery,KeyFrame* mpMatched)
{    
    double duration = static_cast<double>(cv::getTickCount());
    ORBmatcher  mORBmatcher(0.75,true);
    vector<MapPoint*> vvpMapPointMatches;
    vector< cv::DMatch > matchestest;
    int nmatches = mORBmatcher.SearchByBoW(mpQuery,mpMatched,vvpMapPointMatches);
    std::cout<<"mORBmatcher return:"<<nmatches<<std::endl;
    std::cout<<"matchestest return:"<<matchestest.size()<<std::endl;


    duration = static_cast<double>(cv::getTickCount());

    vector< cv::DMatch > matches;
//    cv::FlannBasedMatcher matcher(new cv::flann::LshIndexParams(20, 10, 2));//more slow around:0.15s
    cv::BFMatcher matcher(cv::NORM_HAMMING,false);//around 0.005s,set true,enable cross check cann't be enable with ratiotest

//    matcher.match( descriptors_1, descriptors_2, matches );
    getMatches(matcher,mpQuery->mDescriptors,mpMatched->mDescriptors,matches);
    std::cout<<"before ByMinDistance matches2 size:"<<matches.size()<<std::endl;

    //test function rum time
    duration = static_cast<double>(cv::getTickCount())-duration;
    duration /= (double)cv::getTickFrequency();//unit "s"
    cout<<" OPENCV orb matches rum time:"<<duration<<" S"<<endl;
    ByMinDistance(matches,4);

    std::cout<<"before refine matches2 size:"<<matches.size()<<std::endl;

#if 1
    estimateMotion(matches,mpQuery->mK,mpQuery->mvKeysUn,mpMatched->mvKeysUn,mpQuery->mvDepth,mpMatched->mvDepth);

#else
    cv::Mat m_refinedHomography;
    refineMatchesWithHomography(mpQuery->mvKeys,mpMatched->mvKeys,2,matches,m_refinedHomography);
#endif

    std::cout<<"final goodMatches size:"<<matches.size()<<std::endl;
    //test function rum time
    duration = static_cast<double>(cv::getTickCount())-duration;
    duration /= (double)cv::getTickFrequency();//unit "s"
    cout<<" my orb matches rum time:"<<duration<<" S"<<endl;

    if (matches.size() <= 8)
    {
        return;
    }

    std::vector<Eigen::Vector3d> inlQurey;
    std::vector<Eigen::Vector3d> inlMatched;
    projectInlierMatches(matches,
                              inlQurey,
                              inlMatched,
                              mpQuery,
                              mpMatched);

    std::cout<<"inlQurey.size:"<<inlQurey.size()<<" inlMatched.size:"<<inlMatched.size()<<std::endl;

//    // -- dwaw matches
//    cv::Mat img_goodmatches;
//    cv::drawMatches(img_1, mpQuery->mvKeys, img_2,mpMatched->mvKeys,matches, img_goodmatches);
//    // -- show
//    cv::imshow("goodMathces", img_goodmatches);
//    cv::waitKey(0);
}
void ORB3DTools::timeTest(KeyFrame* mpQuery,KeyFrame* mpMatched)
{
    int num = 10;
    int detectionNSum=0;
    int inlinerNSum=0;
    double durationSum =0;
    double duration1Sum =0;
    for(int i=0;i<num;i++)
    {
        double duration = static_cast<double>(cv::getTickCount());
        double duration1 = static_cast<double>(cv::getTickCount());
        vector< cv::DMatch > matches;
        ORBmatcher  mORBmatcher(0.75,true);
        vector<MapPoint*> vvpMapPointMatches;
        int nmatches = mORBmatcher.mySearchByBoW(mpQuery,mpMatched,matches,vvpMapPointMatches);
        detectionNSum += matches.size();
        //test function rum time
        duration = static_cast<double>(cv::getTickCount())-duration;
        duration /= (double)cv::getTickFrequency();//unit "s"
        durationSum += duration;

    #if 1
        estimateMotion(matches,mpQuery->mK,mpQuery->mvKeysUn,mpMatched->mvKeysUn,mpQuery->mvDepth,mpMatched->mvDepth);
    #else
        cv::Mat m_refinedHomography;
        refineMatchesWithHomography(mpQuery->mvKeys,mpMatched->mvKeys,2,matches,m_refinedHomography);
    #endif

        inlinerNSum += matches.size();
        //test function rum time
        duration1 = static_cast<double>(cv::getTickCount())-duration1;
        duration1 /= (double)cv::getTickFrequency();//unit "s"
        duration1Sum += duration1;
    }
    std::cout<<"mySearchByBoW test results:"<<std::endl;
    std::cout<<"detectionNSum:"<<detectionNSum<<std::endl;
    std::cout<<"inlinerNSum:"<<inlinerNSum<<std::endl;
    std::cout<<"inliner ratio:"<<(float)inlinerNSum/detectionNSum<<std::endl;
    std::cout<<"durationSum:"<<durationSum<<std::endl;
    std::cout<<"duration1Sum:"<<duration1Sum<<std::endl;


    detectionNSum=0;
    inlinerNSum=0;
    durationSum =0;
    duration1Sum =0;
    for(int i=0;i<num;i++)
    {
        double duration = static_cast<double>(cv::getTickCount());
        double duration1 = static_cast<double>(cv::getTickCount());
        vector< cv::DMatch > matches;
    //    cv::FlannBasedMatcher matcher(new cv::flann::LshIndexParams(20, 10, 2));//more slow around:0.15s
        cv::BFMatcher matcher(cv::NORM_HAMMING,false);//around 0.005s,set true,enable cross check cann't be enable with ratiotest

        getMatches(matcher,mpQuery->mDescriptors,mpMatched->mDescriptors,matches);
        detectionNSum += matches.size();
        //test function rum time
        duration = static_cast<double>(cv::getTickCount())-duration;
        duration /= (double)cv::getTickFrequency();//unit "s"
        durationSum += duration;
//        ByMinDistance(matches,4);
    #if 1
        estimateMotion(matches,mpQuery->mK,mpQuery->mvKeysUn,mpMatched->mvKeysUn,mpQuery->mvDepth,mpMatched->mvDepth);
    #else
        cv::Mat m_refinedHomography;
        refineMatchesWithHomography(mpQuery->mvKeys,mpMatched->mvKeys,2,matches,m_refinedHomography);
    #endif

        inlinerNSum += matches.size();
        //test function rum time
        duration1 = static_cast<double>(cv::getTickCount())-duration1;
        duration1 /= (double)cv::getTickFrequency();//unit "s"
        duration1Sum += duration1;
    }
    std::cout<<"BFMatcher test results:"<<std::endl;
    std::cout<<"detectionNSum:"<<detectionNSum<<std::endl;
    std::cout<<"inlinerNSum:"<<inlinerNSum<<std::endl;
    std::cout<<"inliner ratio:"<<(float)inlinerNSum/detectionNSum<<std::endl;
    std::cout<<"durationSum:"<<durationSum<<std::endl;
    std::cout<<"duration1Sum:"<<duration1Sum<<std::endl;

}


void ORB3DTools::ORBmatches(KeyFrame* mpQuery,KeyFrame* mpMatched)
{
//    timeTest(mpQuery,mpMatched);
    std::cout<<"mORBmatcher:"<<std::endl;

    double duration = static_cast<double>(cv::getTickCount());
    vector< cv::DMatch > matches;
#if 1
    ORBmatcher  mORBmatcher(0.75,true);
    vector<MapPoint*> vvpMapPointMatches;
    int nmatches = mORBmatcher.mySearchByBoW(mpQuery,mpMatched,matches,vvpMapPointMatches);
    std::cout<<"mORBmatcher return:"<<nmatches<<std::endl;
    std::cout<<"matchestest return:"<<matches.size()<<std::endl;
    //test function rum time
    duration = static_cast<double>(cv::getTickCount())-duration;
    duration /= (double)cv::getTickFrequency();//unit "s"
    cout<<" my orb matches rum time:"<<duration<<" S"<<endl;

#else

//    cv::FlannBasedMatcher matcher(new cv::flann::LshIndexParams(20, 10, 2));//more slow around:0.15s
    cv::BFMatcher matcher(cv::NORM_HAMMING,false);//around 0.005s,set true,enable cross check cann't be enable with ratiotest

    getMatches(matcher,mpQuery->mDescriptors,mpMatched->mDescriptors,matches);
    //    matcher.match(mpQuery->mDescriptors,mpMatched->mDescriptors,matches);
    std::cout<<"before ByMinDistance matches size:"<<matches.size()<<std::endl;
    ByMinDistance(matches,4);


    std::cout<<"before refine matches2 size:"<<matches.size()<<std::endl;
#endif
    duration = static_cast<double>(cv::getTickCount());
#if 1
    estimateMotion(matches,mpQuery->mK,mpQuery->mvKeysUn,mpMatched->mvKeysUn,mpQuery->mvDepth,mpMatched->mvDepth);
#else
    cv::Mat m_refinedHomography;
    refineMatchesWithHomography(mpQuery->mvKeys,mpMatched->mvKeys,2,matches,m_refinedHomography);
#endif
    std::cout<<"final goodMatches size:"<<matches.size()<<std::endl;
    //test function rum time
    duration = static_cast<double>(cv::getTickCount())-duration;
    duration /= (double)cv::getTickFrequency();//unit "s"
    cout<<" my orb matches rum time:"<<duration<<" S"<<endl;

    if (matches.size() <= 8)
    {
        return;
    }

    std::vector<Eigen::Vector3d> inlQurey;
    std::vector<Eigen::Vector3d> inlMatched;
    projectInlierMatches(matches,
                              inlQurey,
                              inlMatched,
                              mpQuery,
                              mpMatched);

    std::cout<<"inlQurey.size:"<<inlQurey.size()<<" inlMatched.size:"<<inlMatched.size()<<std::endl;


    Eigen::Matrix4f relativeTransform;
    Eigen::Matrix4f eigenQuery = convertToEigen4f(mpQuery);
    Eigen::Matrix4f eigenMatched = convertToEigen4f(mpMatched);
    relativeTransform = eigenQuery.inverse() * eigenMatched;//good result: compare iSAM->addCameraCameraConstraint  and  iSAM->addLoopConstraint, should find this is right.
//    relativeTransform = eigenMatched * eigenQuery.inverse();//not so good,above is right
    mpDeformation->SetORBLoopCloureMatches(mpQuery->mTimeStamp*1000000,
                                           mpMatched->mTimeStamp*1000000,
                                           relativeTransform,
                                           inlQurey,
                                           inlMatched);
}

void ORB3DTools::getMatches(cv::BFMatcher matcher,const cv::Mat& queryDesc,
                            const cv::Mat & matchedDesc,std::vector<cv::DMatch>& matches)
{
    matches.clear();

    bool enableRatioTest = true;
    if (enableRatioTest)
    {
        // To avoid NaN's when best match has zero distance we will use inversed ratio.
        const float minRatio = 1.f / 1.25f;

        // KNN match will return 2 nearest matches for each query descriptor
        std::vector< std::vector<cv::DMatch> > m_knnMatches;
        matcher.knnMatch(queryDesc,matchedDesc, m_knnMatches, 2);

        for (size_t i=0; i<m_knnMatches.size(); i++)
        {
            const cv::DMatch& bestMatch   = m_knnMatches[i][0];
            const cv::DMatch& betterMatch = m_knnMatches[i][1];
#if 0
            float distanceRatio = bestMatch.distance / std::max(betterMatch.distance,float(0.01));

            // Pass only matches where distance ratio between
            // nearest matches is greater than 1.5 (distinct criteria)
            if (distanceRatio < minRatio)
            {
                matches.push_back(bestMatch);
            }
#else
            if((bestMatch.distance < 50) && (bestMatch.distance<0.75*betterMatch.distance))
                matches.push_back(bestMatch);
#endif
        }
    }
    else
    {
        // Perform regular match
        matcher.match(queryDesc,matchedDesc, matches);
    }
}


int ORB3DTools::estimateMotion( std::vector<cv::DMatch>  &goodMatches,const cv::Mat mk,
                    const std::vector<cv::KeyPoint> mvKeysquery,const std::vector<cv::KeyPoint> mvKeysmatched,
                    const std::vector<float> mvDepthQuery,const std::vector<float> mvDepthMatched)
{
    // query
    vector<cv::Point3f> pts_obj;
    // matched
    vector< cv::Point2f > pts_img;

    float fx = mk.at<float>(0,0);
    float fy = mk.at<float>(1,1);
    float cx = mk.at<float>(0,2);
    float cy = mk.at<float>(1,2);

    for (size_t i=0; i<goodMatches.size(); i++)
    {
        cv::Point2f p = mvKeysquery[goodMatches[i].queryIdx].pt;
        // be careful ,y is rows，x is cols！
        float d = mvDepthQuery[goodMatches[i].queryIdx];//the depth in keyframe is already scaled
        if (d < 0)
            continue;
        pts_img.push_back( cv::Point2f( mvKeysmatched[goodMatches[i].trainIdx].pt ) );
        // 将(u,v,d)转成(x,y,z)
        cv::Point3f pt ( p.x, p.y, d );
        cv::Point3f pd = point2dTo3d( pt,fx,fy,cx,cy);
        pts_obj.push_back( pd );
    }


    if (pts_obj.size() ==0 || pts_img.size()==0)
    {
        return -1;
    }

    cv::Mat rvec, tvec, inliers;
//    cv::solvePnPRansac( pts_obj, pts_img, mk, cv::Mat(), rvec, tvec, false, 100, 1.0, 100, inliers );
    cv::solvePnPRansac( pts_obj, pts_img, mk, cv::Mat(), rvec, tvec, false, 100, 2.0, 0.85, inliers );
    std::vector<cv::DMatch> matches;
    for(int i=0;i<inliers.rows;i++)
    {
        int n = inliers.at<int>(i);
        if(n)
            matches.push_back(goodMatches[n]);
    }
//    std::cout<<"inliers/ratio:"<<(float)matches.size()/goodMatches.size()<<std::endl;
    goodMatches.swap(matches);
//    std::cout<<"inliers/ratio after swap:"<<(float)matches.size()/goodMatches.size()<<std::endl;
    return inliers.rows;
}

cv::Point3f ORB3DTools::point2dTo3d( cv::Point3f& point,const float fx,const float fy,const float cx,const float cy)
{
    int scale = 1;//already scaled
    cv::Point3f p; // 3D 点
    p.z = double( point.z ) / scale;
    p.x = ( point.x - cx) * p.z / fx;
    p.y = ( point.y - cy) * p.z / fy;
    return p;
}

bool ORB3DTools::refineMatchesWithHomography(const std::vector<cv::KeyPoint>& queryKeypoints,
                                             const std::vector<cv::KeyPoint>& trainKeypoints,
                                             float reprojectionThreshold,
                                             std::vector<cv::DMatch>& matches,
                                             cv::Mat& homography
                                             )
{
    const int minNumberMatchesAllowed = 8;

    if (matches.size() < minNumberMatchesAllowed)
        return false;

    // Prepare data for cv::findHomography
    std::vector<cv::Point2f> srcPoints(matches.size());
    std::vector<cv::Point2f> dstPoints(matches.size());

    for (size_t i = 0; i < matches.size(); i++)
    {
        srcPoints[i] = trainKeypoints[matches[i].trainIdx].pt;
        dstPoints[i] = queryKeypoints[matches[i].queryIdx].pt;
    }

    // Find homography matrix and get inliers mask
    std::vector<unsigned char> inliersMask(srcPoints.size());
    homography = cv::findHomography(srcPoints,
                                    dstPoints,
                                    CV_FM_RANSAC,
                                    reprojectionThreshold,
                                    inliersMask);

    std::vector<cv::DMatch> inliers;
    for (size_t i=0; i<inliersMask.size(); i++)
    {
        if (inliersMask[i])
            inliers.push_back(matches[i]);
    }
    std::cout<<"inliers/ratio:"<<(float)inliers.size()/matches.size()<<std::endl;
    matches.swap(inliers);
    std::cout<<"inliers/ratio after swap:"<<(float)inliers.size()/matches.size()<<std::endl;
    return matches.size() > minNumberMatchesAllowed;
}


void ORB3DTools::projectInlierMatches(std::vector<cv::DMatch>& goodMatches,
                                       std::vector<Eigen::Vector3d> & inlQurey,
                                       std::vector<Eigen::Vector3d> & inlMatched,
                                       KeyFrame* mpQuery,
                                       KeyFrame* mpMatched)
{
    cv::Mat mk = mpQuery->mK;
    float fx = mk.at<float>(0,0);
    float fy = mk.at<float>(1,1);
    float cx = mk.at<float>(0,2);
    float cy = mk.at<float>(1,2);

    inlQurey.clear();
    inlMatched.clear();

    for (size_t i=0; i<goodMatches.size(); i++)
    {
        // be careful ,y is rows，x is cols！
        float depthQuery = mpQuery->mvDepth[goodMatches[i].queryIdx];//the depth in keyframe is already scaled
        float depthMatched = mpMatched->mvDepth[goodMatches[i].trainIdx];//the depth in keyframe is already scaled
        if(!depthQuery || !depthMatched)
        {
            continue;
        }
        cv::Point2f pQuery = mpQuery->mvKeysUn[goodMatches[i].queryIdx].pt;
        cv::Point2f pMatched = mpMatched->mvKeysUn[goodMatches[i].trainIdx].pt;
        // 将(u,v,d)转成(x,y,z)
        cv::Point3f ptQuery ( pQuery.x, pQuery.y, depthQuery );
        cv::Point3f ptMatched ( pMatched.x, pMatched.y, depthMatched );
        inlQurey.push_back(point2dToEigen3d(ptQuery,fx,fy,cx,cy));
        inlMatched.push_back(point2dToEigen3d(ptMatched,fx,fy,cx,cy));
    }
}

Eigen::Vector3d ORB3DTools::point2dToEigen3d( cv::Point3f& point,const float fx,const float fy,const float cx,const float cy)
{
    int scale = 1;//already scaled
    Eigen::Vector3d p; // 3D 点
    p[2] = double( point.z ) / scale;
    p[0] = ( point.x - cx) * p[2] / fx;
    p[1] = ( point.y - cy) * p[2] / fy;
    return p;
}

Eigen::Matrix4f ORB3DTools::convertToEigen4f(KeyFrame* mKeyframe)
{
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    cv::Mat mTcw = mKeyframe->GetPose();
    cv::Mat Rwc = mTcw.rowRange(0,3).colRange(0,3).t();
    cv::Mat twc = -Rwc*mTcw.rowRange(0,3).col(3);
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
        {
            transform(i,j) = Rwc.at<float>(i,j);
        }
    for(int i=0;i<3;i++)
        transform(i,3) = twc.at<float>(i);

    return transform;
}


} //namespace ORB_SLAM
