/*
 * This file is part of ORBFusion.
 */

#include "MainController.h"

#include <boost/filesystem.hpp>
//#include <boost/algorithm/algorithm.hpp>
#include <boost/detail/algorithm.hpp> //for ubuntu 14.04 --Justin
#include <boost/algorithm/string.hpp>


MainController * MainController::controller = 0;

MainController::MainController(int argc, char * argv[])
 : depthIntrinsics(0),
   pangoVis(0),
   trackerInterface(0),
   meshGenerator(0),
   placeRecognition(0),
   cloudSliceProcessor(0),
   deformation(0),
   slamEngineRead(0),
   logRead(0)

{
    ConfigArgs::get(argc, argv);

    assert(!MainController::controller);

    MainController::controller = this;

    directoryPath="/media/djt/share/dataset/TUMDataset/dyson_lab";
    ORBvocPath="../src/SlamEngine/Vocabulary/ORBvoc.bin";
    cameraParaterPath="../loop.yaml";
}

MainController::~MainController()
{
    if(depthIntrinsics)
    {
        delete depthIntrinsics;
    }
}

int MainController::start()
{
    if(setup())
    {
        return mainLoop();
    }
    else
    {
        return -1;
    }
}

bool MainController::setup()
{

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    Volume::get(ConfigArgs::get().volumeSize);

    Stopwatch::get().setCustomSignature(43543534);

    cudaSafeCall(cudaSetDevice(ConfigArgs::get().gpu));

    loadCalibration();

    std::cout << "Point resolution: " << ((int)((Volume::get().getVoxelSizeMeters().x * 1000.0f) * 10.0f)) / 10.0f << " millimetres" << std::endl;


    slamEngineRead = new SlamEngineLogReader;
    logRead = static_cast<LogReader *>(slamEngineRead);
    logRead->setDepthFactor(mDepthMapFactor);


    ThreadDataPack::get();

    trackerInterface = new TrackerInterface(logRead, depthIntrinsics);

    if(ConfigArgs::get().trajectoryFile.size())
    {
        std::cout << "Load trajectory: " << ConfigArgs::get().trajectoryFile << std::endl;
        trackerInterface->loadTrajectory(ConfigArgs::get().trajectoryFile);
    }

    systemComponents.push_back(trackerInterface);

    ThreadDataPack::get().assignFrontend(trackerInterface->getFrontend());

    cloudSliceProcessor = new CloudSliceProcessor();
    systemComponents.push_back(cloudSliceProcessor);

    if(ConfigArgs::get().extractOverlap)
    {
        trackerInterface->enableOverlap();
    }

    if(!ConfigArgs::get().incrementalMesh && ConfigArgs::get().enableMeshGenerator)
    {
        meshGenerator = new MeshGenerator();
        systemComponents.push_back(meshGenerator);
    }
    else
    {
        ThreadDataPack::get().meshGeneratorFinished.assignValue(true);
    }


//    deformation = new Deformation;
//    systemComponents.push_back(deformation);
    if(ConfigArgs::get().vocabFile.size() && ConfigArgs::get().onlineDeformation)
    {
        placeRecognition = new PlaceRecognition(depthIntrinsics);
        systemComponents.push_back(placeRecognition);

        deformation = new Deformation;
        systemComponents.push_back(deformation);
    }
    else
    {
        ThreadDataPack::get().deformationFinished.assignValue(true);
        ThreadDataPack::get().placeRecognitionFinished.assignValue(true);
    }

    pangoVis = new PangoVis(depthIntrinsics);

    return true;
}

int MainController::mainLoop()
{
    //for slamEngine
    SlamEngine *mSlamEngine = new SlamEngine(directoryPath,cameraParaterPath,ORBvocPath,
                                             placeRecognition,deformation,slamEngineRead);
    boost::thread slamEngineTread(boost::bind(&SlamEngine::process, mSlamEngine));

    std::cout<<"start main"<<std::endl;

    timeval start;
    gettimeofday(&start, 0);
    uint64_t beginning = start.tv_sec * 1000000 + start.tv_usec;

    for(unsigned int i = 0; i < systemComponents.size(); i++)
    {
        threads.add_thread(new boost::thread(boost::bind(&ThreadObject::start, systemComponents.at(i))));
    }
    if(pangoVis)
    {
        pangoVis->start();
    }

    slamEngineTread.join();
    threads.join_all();


    for(unsigned int i = 0; i < systemComponents.size(); i++)
    {
        delete systemComponents.at(i);
    }

    if(pangoVis)
    {
        pangoVis->stop();
        delete pangoVis;
    }


    return 0;
}

void MainController::loadCalibration()
{
    cv::FileStorage fSettings(cameraParaterPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    depthIntrinsics = new cv::Mat(cv::Mat::zeros(3, 3, CV_64F));

    depthIntrinsics->at<double>(0, 2) = cx;
    depthIntrinsics->at<double>(1, 2) = cy;
    depthIntrinsics->at<double>(0, 0) = fx;
    depthIntrinsics->at<double>(1, 1) = fy;
    depthIntrinsics->at<double>(2, 2) = 1;


    mDepthMapFactor = fSettings["DepthMapFactor"];

    std::cout<<"depthIntrinsics:"<<std::endl;
    std::cout<<"cx:"<<depthIntrinsics->at<double>(0, 2)<<std::endl;
    std::cout<<"cy:"<<depthIntrinsics->at<double>(1, 2)<<std::endl;
    std::cout<<"fx:"<<depthIntrinsics->at<double>(0, 0)<<std::endl;
    std::cout<<"fy:"<<depthIntrinsics->at<double>(1, 1)<<std::endl;

    std::cout<<"mDepthMapFactor:"<<mDepthMapFactor<<std::endl;
    Resolution::get(640, 480);
}

void MainController::complete()
{
    trackerInterface->endRequested.assignValue(true);
}

void MainController::save()
{
    if(ThreadDataPack::get().finalised.getValue())
    {
        if(!ConfigArgs::get().onlineDeformation)
        {
            boost::thread * cloudSaveThread = new boost::thread(boost::bind(&CloudSliceProcessor::save, cloudSliceProcessor));
            assert(cloudSaveThread);

            if(controller->meshGenerator)
            {
                boost::thread * meshSaveThread = new boost::thread(boost::bind(&MeshGenerator::save, meshGenerator));
                assert(meshSaveThread);
            }
        }
        else
        {
            boost::thread * cloudSaveThread = new boost::thread(boost::bind(&Deformation::saveCloud, deformation));
            assert(cloudSaveThread);

            if(ConfigArgs::get().enableMeshGenerator)
            {
                boost::thread * meshSaveThread = new boost::thread(boost::bind(&Deformation::saveMesh, deformation));
                assert(meshSaveThread);
            }
        }
    }
    else
    {
         std::cout<<"MainController::save:failed"<<std::endl;
    }
}

void MainController::reset()
{
    if(!ThreadDataPack::get().finalised.getValue())
    {
        for(unsigned int i = 0; i < systemComponents.size(); i++)
        {
            systemComponents.at(i)->stop();
        }

        while(true)
        {
            bool stillRunning = false;

            for(unsigned int i = 0; i < systemComponents.size(); i++)
            {
                if(systemComponents.at(i)->running())
                {
                    stillRunning = true;
                    break;
                }
            }

            if(!stillRunning)
            {
                break;
            }
            else
            {
                ThreadDataPack::get().notifyVariables();
                ThreadDataPack::get().tracker->cloudSignal.notify_all();
            }
        }

        threads.join_all();

        if(pangoVis)
        {
            pangoVis->reset();
        }

        for(unsigned int i = 0; i < systemComponents.size(); i++)
        {
            systemComponents.at(i)->reset();
        }

        ThreadDataPack::get().reset();

        for(unsigned int i = 0; i < systemComponents.size(); i++)
        {
            threads.add_thread(new boost::thread(boost::bind(&ThreadObject::start, systemComponents.at(i))));
        }
    }
}

void MainController::setPark(const bool park)
{
    trackerInterface->setPark(park);
}

void MainController::shutdown()
{
    for(size_t i = 0; i < systemComponents.size(); i++)
    {
        systemComponents.at(i)->stop();
    }

    while(true)
    {
        bool stillRunning = false;

        for(size_t i = 0; i < systemComponents.size(); i++)
        {
            if(systemComponents.at(i)->running())
            {
                stillRunning = true;
                break;
            }
        }

        if(!stillRunning)
        {
            break;
        }
        else
        {
            ThreadDataPack::get().notifyVariables();
            ThreadDataPack::get().tracker->cloudSignal.notify_all();
        }
    }

    if(pangoVis)
    {
        pangoVis->stop();
    }
}

uint64_t MainController::getMaxLag()
{
    uint64_t maxLag = 0;

    for(size_t i = 0; i < systemComponents.size(); i++)
    {
        maxLag = std::max(systemComponents.at(i)->lagTime.getValue(), maxLag);
    }

    return maxLag;
}


