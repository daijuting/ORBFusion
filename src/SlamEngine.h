/*
 * This file is part of ORBFusion.
 */

#ifndef SLAMENGINE_H
#define SLAMENGINE_H

#include <iostream>
#include <unistd.h>
#include <fstream>
#include <opencv2/opencv.hpp>

#include<algorithm>
#include<chrono>

#include "SlamEngine/include/System.h"

#include "utils/SlamEngineLogReader.h"
#include "backend/Deformation.h"
#include "backend/PlaceRecognition.h"


struct GroundTruthPose
{
    cv::Mat mTcw;
    double vTime;
    bool mTrackState;
};

class SlamEngine{

public:
    //for orb-slam2
    SlamEngine(const string mdatasetDirectory,const string mcameraParameter,const string mORBvocPath,
               PlaceRecognition *_mPlaceRecognition,Deformation *_mDeformation,SlamEngineLogReader * _mSlamEngineLogReader);
    ~SlamEngine();
    void process();

private:
    SlamEngineLogReader * mSlamEngineLogReader;
    Deformation *mDeformation;
    PlaceRecognition *mPlaceRecognition;

    // Retrieve paths to images
    std::vector<std::string> vstrImageFilenamesRGB;
    std::vector<std::string> vstrImageFilenamesD;
    std::vector<double> vTimestamps;
    std::string directoryPath;
    std::string cameraParaterPath;
    std::string ORBvocPath;


    std::vector<GroundTruthPose> vGroundTruthPose;

    void LoadTUMImages(const std::string &strAssociationFilename, std::vector<std::string> &vstrImageFilenamesRGB,
                    std::vector<std::string> &vstrImageFilenamesD, std::vector<double> &vTimestamps);

    void LoadloopImages(const std::string &strAssociationFilename, std::vector<std::string> &vstrImageFilenamesRGB,
                    std::vector<std::string> &vstrImageFilenamesD, std::vector<double> &vTimestamps);

    void LoadICL_NUIMImages(const std::string &strAssociationFilename, std::vector<std::string> &vstrImageFilenamesRGB,
                    std::vector<std::string> &vstrImageFilenamesD, std::vector<double> &vTimestamps);

    void loadTrajectory(const std::string & filename);
    void printGroundTruthPose(const GroundTruthPose pose);

};

#endif // SLAMEIGINE_H
