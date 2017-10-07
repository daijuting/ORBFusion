/*
 * This file is part of ORBFusion.
 */

#include "SlamEngineLogReader.h"

SlamEngineLogReader::SlamEngineLogReader()
{
    compressedDepth = new unsigned char[Resolution::get().numPixels() * 2];
    compressedImage = new unsigned char[Resolution::get().numPixels() * 3];


    decompressionBuffer = new Bytef[Resolution::get().numPixels() * 2];
    deCompImage = cvCreateImage(cvSize(Resolution::get().width(), Resolution::get().height()), IPL_DEPTH_8U, 3);

    isCompressed = false;
    currentFrame = 0;
    mbFinished = false;

    decompressedDepth = new unsigned short[Resolution::get().numPixels()];
}

SlamEngineLogReader::~SlamEngineLogReader()
{
    delete [] compressedDepth;
    delete [] compressedImage;
    delete [] decompressionBuffer;
}

void SlamEngineLogReader::testrun()
{
    bool returnVal=0;
    while(grabNext(returnVal, currentFrame))
    {

    }
    std::cout<<"read finished"<<std::endl;
}

void SlamEngineLogReader::readNext()
{
    cv::Mat imRGB, imD,imDisRGB;

    // Read image and depthmap vTimestamps from list
    {
        std::unique_lock<std::mutex> lck(frameMutex);
        imDisRGB = rgbImgs.front();
        rgbImgs.pop_front();
        imD = depthImgs.front();
        depthImgs.pop_front();

        double timeTmp =vTimestamps.front();
        timestamp = int64_t(timeTmp*1000000);
        vTimestamps.pop_front();

        mTcw = mTcws.front();
        mTcws.pop_front();

        isLoop = isLoops.front();
        isLoops.pop_front();

        if(rgbImgs.size()>2)
            std::cout<<"SlamEngineLogReader rgbImgs SIZE:"<<rgbImgs.size()<<std::endl;
//        std::cout<<"receive depthImgs SIZE after:"<<depthImgs.size()<<std::endl;
//        std::cout<<"receive vTimestamps SIZE after:"<<vTimestamps.size()<<std::endl;
//        std::cout<<"receive mTcws SIZE after:"<<mTcws.size()<<std::endl;

    }

    if(mDistCoef.at<float>(0)==0.0)
    {
        imRGB = imDisRGB;
    }
    else
    {
        cv::undistort(imDisRGB,imRGB,mK,mDistCoef,mK);
    }
    if(!imRGB.isContinuous())
        std::cout<<"imRGB is NOT continuous"<<std::endl;
    if(!imD.isContinuous())
        std::cout<<"imD is NOT continuous"<<std::endl;

    isCompressed = false;

    memcpy(&decompressionBuffer[0], imD.data, Resolution::get().numPixels() * 2);
    decompressedDepth = (unsigned short *)&decompressionBuffer[0];

    if(mDepthFactor!=1000)
    {
//        std::cout<<mDepthFactor<<std::endl;
        float scale= mDepthFactor/1000;
        for(int i=0;i<Resolution::get().numPixels();i++)
        {
            decompressedDepth[i]=decompressedDepth[i]/scale;
        }
    }


//    for(int i=0;i<imD.rows;i++)
//        for(int j=0;j<imD.cols;j++)
//        {
//            decompressedDepth[i*imD.cols +j] = imD.ptr<unsigned short>(i)[j]/5;
//        }

    memcpy(deCompImage->imageData, imRGB.data, Resolution::get().numPixels() * 3);
    decompressedImage = (unsigned char *)deCompImage->imageData;

    compressedImage = 0;
    compressedDepth = 0;

    compressedImageSize = Resolution::get().numPixels() * 3;
    compressedDepthSize = Resolution::get().numPixels() * 2;

    if(ConfigArgs::get().flipColors)
    {
        cv::Mat3b rgb(Resolution::get().rows(),
                      Resolution::get().cols(),
                      (cv::Vec<unsigned char, 3> *)deCompImage->imageData,
                      Resolution::get().width() * 3);

        cv::cvtColor(rgb, rgb, CV_RGB2BGR);

    }

    currentFrame++;
}

bool SlamEngineLogReader::grabNext(bool & returnVal, int & currentFrame)
{
    //check whether orb-slam is finished
    if(isFinished())
    {
        returnVal=false;
        return false;
    }

    if(rgbImgs.size()>0 && depthImgs.size()>0)
    {
        readNext();
        ThreadDataPack::get().trackerFrame.assignAndNotifyAll(currentFrame);
        return true;
    }
    //wait for the next frame sent from tracking thread by orb-slam
    {
        std::unique_lock<std::mutex> lck_keyframeUpdated( frameUpdateMutex );
        frameUpdated.wait( lck_keyframeUpdated );
    }
    if(rgbImgs.size()>0 && depthImgs.size()>0)
    {
        readNext();
        ThreadDataPack::get().trackerFrame.assignAndNotifyAll(currentFrame);
        return true;
    }
    returnVal = false;
    return false;
}

void SlamEngineLogReader::insertFrame(double _vTimestamps,cv::Mat &rgb, cv::Mat &depth,cv::Mat &mTcw, bool loopFlage)
{

    std::unique_lock<std::mutex> lck(frameMutex);
    rgbImgs.push_back( rgb);
    depthImgs.push_back( depth );
    vTimestamps.push_back(_vTimestamps);
    mTcws.push_back(mTcw);
    isLoops.push_back(loopFlage);
    frameUpdated.notify_one();
//    std::cout<<"receive frame its size:"<<rgbImgs.size()<<std::endl;
}

void SlamEngineLogReader::SetFinish()
{
    std::unique_lock<std::mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool SlamEngineLogReader::isFinished()
{
    std::unique_lock<std::mutex> lock(mMutexFinish);
    return mbFinished;
}
bool SlamEngineLogReader::printHello()
{
    std::cout<<"SlamEngineLogReader hello"<<rgbImgs.size()<<std::endl;
}

void SlamEngineLogReader::SetCameraParameter(cv::Mat _mK,cv::Mat _mDistCoef)
{
    mK = _mK.clone();
    mDistCoef = _mDistCoef.clone();
    std::cout<<"set SetCameraParameter for SlamEngineLogReader"<<std::endl;
    if(mDistCoef.at<float>(0)==0.0)
    {
        std::cout<<"the image is undistorted"<<std::endl;
    }
    else
    {
        std::cout<<"the image is distorted"<<std::endl;
    }
}
