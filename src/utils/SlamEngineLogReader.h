/*
 * This file is part of ORBFusion.
 */

#ifndef SLAMENGINELOGREADER_H_
#define SLAMENGINELOGREADER_H_

#include <boost/filesystem.hpp>
#include <condition_variable>
#include <mutex>

#include <iostream>
#include <vector>

#include "LogReader.h"

class SlamEngineLogReader : public LogReader
{
    public:
        SlamEngineLogReader();

        virtual ~SlamEngineLogReader();

        bool grabNext(bool & returnVal, int & currentFrame);
        void insertFrame(double _vTimestamps,cv::Mat &rgb, cv::Mat &depth,cv::Mat &mTcw, bool loopFlage);
        bool printHello();

        void SetFinish();
        bool isFinished();
        void testrun();

        void SetCameraParameter(cv::Mat _mK,cv::Mat _mDistCoef);


        void setDepthFactor();

    private:

        bool hasMore();
        void readNext();

        void getCore();
        void getNext();


        std::list<cv::Mat> rgbImgs;
        std::list<cv::Mat> depthImgs;
        std::list<double> vTimestamps;
        std::list<bool> isLoops;
        // Camera pose.
        std::list<cv::Mat> mTcws;

        std::condition_variable  frameUpdated;
        std::mutex  frameUpdateMutex;
        std::mutex  frameMutex;

        bool    mbFinished = false;
        std::mutex   mMutexFinish;

        int currentFrame;

        cv::Mat mK;
        cv::Mat mDistCoef;


};

#endif /* SLAMENGINELOGREADER_H_ */
