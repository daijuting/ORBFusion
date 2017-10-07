/*
 * This file is part of ORBFusion.
 */

#include "SlamEngine.h"

SlamEngine::SlamEngine(const string mdatasetDirectory,const string mcameraParameter,const string mORBvocPath,
           PlaceRecognition *_mPlaceRecognition,Deformation *_mDeformation,SlamEngineLogReader * _mSlamEngineLogReader)
    :mPlaceRecognition(_mPlaceRecognition),mDeformation(_mDeformation),mSlamEngineLogReader(_mSlamEngineLogReader)
{
    std::cout<<"enter SlamEngine"<<std::endl;
    directoryPath = mdatasetDirectory;
    ORBvocPath = mORBvocPath;
    cameraParaterPath = mcameraParameter;
    std::string strAssociationFilename=directoryPath + "/associations.txt";

    std::cout<<strAssociationFilename<<std::endl;

    LoadTUMImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);


    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesRGB.size();
    if(vstrImageFilenamesRGB.empty())
    {
        std::cerr << std::endl << "No images found in provided path." << std::endl;
        return ;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        std::cerr << std::endl << "Different number of images for rgb and depth." << std::endl;
        return ;
    }
    std::cout<<"image size:"<<vstrImageFilenamesD.size()<<std::endl;

//    loadTrajectory("GroundtruthForICP_desk2.txt");
    if(vGroundTruthPose.size()!=vstrImageFilenamesD.size())
    {
        std::cerr << std::endl << "Different number of images and ground truth pose" << std::endl;
        return ;
    }

    for(int i=0;i<vGroundTruthPose.size();i++)
    {
        if(vGroundTruthPose[i].vTime != vTimestamps[i])
        {
            std::cerr << std::endl << "Different vTimestamps and ground truth:" << i << std::endl;
            return ;
        }
    }

}
SlamEngine::~SlamEngine()
{
    std::cout<<"exit SlamEngine"<<std::endl;
}

void SlamEngine::process()
{
#define GROUNDTRUTH 0
    int nImages = vstrImageFilenamesRGB.size();

#if 1
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(mPlaceRecognition,mDeformation,mSlamEngineLogReader,ORBvocPath.c_str(),
                           cameraParaterPath.c_str(),ORB_SLAM2::System::RGBD,true);
#else

#endif

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);


    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    //test function rum time
    uint64_t start = Stopwatch::getCurrentSystemTime();

    // Main loop
    cv::Mat imRGB, imD;
    for(int ni=0; ni<nImages; ni++)
    {

        imRGB = cv::imread(std::string(directoryPath)+"/"+vstrImageFilenamesRGB[ni],CV_LOAD_IMAGE_UNCHANGED);
        imD = cv::imread(std::string(directoryPath)+"/"+vstrImageFilenamesD[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(imRGB.empty())
        {
            std::cout<<"image read failed"<<std::endl;
        }
        if(imD.empty())
        {
            std::cout<<"depth read failed"<<std::endl;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif


#if !GROUNDTRUTH
        // Pass the image to the SLAM system
        SLAM.TrackRGBD(imRGB,imD,tframe);
//        usleep(1000*60);
#else
        GroundTruthPose tmp = vGroundTruthPose[ni];
        //insert the groundtruth pose to mSlamEngineLogReader.
        mSlamEngineLogReader->insertFrame(tmp.vTime,imRGB,imD,tmp.mTcw,tmp.mTrackState);
        usleep(1000*70);
#endif



#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;


    }

    //test function rum time
    uint64_t duration = Stopwatch::getCurrentSystemTime() - start;
    cout<<"Stopwatch measured mean rum time with read image from dist:"<<(double)duration/nImages/1000000.0<<" S"<<endl;

    //--Justin
    //this will end the Kintinuous module,otherwise it will wait for the image by mSlamEngineLogReader
    //and cann't save pcd. moreover,it should setFinish() before SLAM.Shutdown(),because if it's after SLAM.Shutdown()
    //it has to wait for the interface of ORB-SLAM all closed, and the code mSlamEngineLogReader->SetFinish(); be
    //implemented,after which the save function of Kintinuous will success.
    //of course,if the interface of ORB-SLAM is disabled, SLAM.Shutdown() can be finished automatically,then
    //the code mSlamEngineLogReader->SetFinish();will be implemented,so will the save function success.
    //in default mode, I will disable the interface of ORB-SLAM. we can enable it in the initialization of ORB_SLAM2::System
//    mSlamEngineLogReader->SetFinish();



    // Stop all threads
    SLAM.Shutdown();


    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages <<" S"<<std::endl;


#if !GROUNDTRUTH
    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("ORB_SLAM-FinalSaved.txt");
//    SLAM.SaveKeyFrameTrajectoryTUM("myKeyFrameTrajectory.txt");
#else
        //nothing

#endif

}

void SlamEngine::LoadTUMImages(const std::string &strAssociationFilename, std::vector<std::string> &vstrImageFilenamesRGB,
                std::vector<std::string> &vstrImageFilenamesD, std::vector<double> &vTimestamps)
{
    std::ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        std::string s;
        std::getline(fAssociation,s);
        if(!s.empty())
        {
            std::stringstream ss;
            ss << s;
            double t;
            std::string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);

        }
    }
}

void SlamEngine::LoadICL_NUIMImages(const std::string &strAssociationFilename, std::vector<std::string> &vstrImageFilenamesRGB,
                std::vector<std::string> &vstrImageFilenamesD, std::vector<double> &vTimestamps)

{
    std::ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        std::string s;
        std::getline(fAssociation,s);
        if(!s.empty())
        {
            std::stringstream ss;
            ss << s;
            double t;
            std::string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);
            ss >> t;
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
        }
    }
}

void SlamEngine::LoadloopImages(const std::string &strAssociationFilename, std::vector<std::string> &vstrImageFilenamesRGB,
                std::vector<std::string> &vstrImageFilenamesD, std::vector<double> &vTimestamps)
{
    std::ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        std::string s;
        std::getline(fAssociation,s);
        if(!s.empty())
        {
            std::stringstream ss;
            ss << s;
            std::string t;
            std::string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(double(std::atoi(t.c_str())));
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);

        }
    }
}


void SlamEngine::printGroundTruthPose(const GroundTruthPose pose)
{
    std::cout<<"printGroundTruthPose"<<std::endl;
    std::cout<< std::setprecision(6) << std::fixed<<"dtime:"<<pose.vTime<<std::endl;
    std::cout<<"mTcw:"<<std::endl;
    std::cout<< std::setprecision(9)<<" "<< pose.mTcw.at<float>(0,0);
    std::cout<<" "<< pose.mTcw.at<float>(0,1);
    std::cout<<" "<< pose.mTcw.at<float>(0,2);
    std::cout<<" "<< pose.mTcw.at<float>(0,3) <<std::endl;
    std::cout<<" "<< pose.mTcw.at<float>(1,0);
    std::cout<<" "<< pose.mTcw.at<float>(1,1);
    std::cout<<" "<< pose.mTcw.at<float>(1,2);
    std::cout<<" "<< pose.mTcw.at<float>(1,3) <<std::endl;
    std::cout<<" "<< pose.mTcw.at<float>(2,0);
    std::cout<<" "<< pose.mTcw.at<float>(2,1);
    std::cout<<" "<< pose.mTcw.at<float>(2,2);
    std::cout<<" "<< pose.mTcw.at<float>(2,3) <<std::endl;
    std::cout<<" "<< pose.mTcw.at<float>(3,0);
    std::cout<<" "<< pose.mTcw.at<float>(3,1);
    std::cout<<" "<< pose.mTcw.at<float>(3,2);
    std::cout<<" "<< pose.mTcw.at<float>(3,3);
    std::cout<<" "<< pose.mTrackState <<std::endl;
}

void SlamEngine::loadTrajectory(const std::string & filename)
{
    std::ifstream file;
    std::string line;
    file.open(filename.c_str());

    std::cout<<"trajectory path name:"<<filename<<std::endl;
    vGroundTruthPose.clear();

    while (!file.eof())
    {
        double dtime;
        cv::Mat mTcw = cv::Mat::eye(4,4,CV_32F);
        bool flage;

        std::getline(file, line);

        std::stringstream ssline;
        ssline << line;

        ssline >> dtime;
        ssline >> mTcw.at<float>(0,0);
        ssline >> mTcw.at<float>(0,1);
        ssline >> mTcw.at<float>(0,2);
        ssline >> mTcw.at<float>(0,3);
        ssline >> mTcw.at<float>(1,0);
        ssline >> mTcw.at<float>(1,1);
        ssline >> mTcw.at<float>(1,2);
        ssline >> mTcw.at<float>(1,3);
        ssline >> mTcw.at<float>(2,0);
        ssline >> mTcw.at<float>(2,1);
        ssline >> mTcw.at<float>(2,2);
        ssline >> mTcw.at<float>(2,3);
        ssline >> mTcw.at<float>(3,0);
        ssline >> mTcw.at<float>(3,1);
        ssline >> mTcw.at<float>(3,2);
        ssline >> mTcw.at<float>(3,3);
        ssline >> flage;

        if(file.eof())
            break;

        GroundTruthPose tmp;
        tmp.mTcw = mTcw;
        tmp.vTime = dtime;
        tmp.mTrackState = flage;

        vGroundTruthPose.push_back(tmp);


    }
    std::cout << "Done loading ground truth, size: " << vGroundTruthPose.size() << std::endl;
    file.close();
}



