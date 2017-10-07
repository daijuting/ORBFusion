/*
 * This file is part of ORBFusion.
 */

#ifndef MAINCONTROLLER_H_
#define MAINCONTROLLER_H_

#include <iostream>
#include <fstream>

#include <GL/glew.h>
#include <zlib.h>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/condition_variable.hpp>

#include <pcl/console/parse.h>
#include <pcl/registration/icp.h>
#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "utils/Stopwatch.h"

#include  "utils/SlamEngineLogReader.h"
#include "utils/LogReader.h"

#include "utils/ThreadDataPack.h"
#include "frontend/cuda/internal.h"
#include "backend/MeshGenerator.h"
#include "backend/PlaceRecognition.h"
#include "backend/CloudSliceProcessor.h"
#include "backend/Deformation.h"
#include "backend/TrackerInterface.h"
#include "utils/ConfigArgs.h"
#include "frontend/Resolution.h"
#include "PangoVis.h"


#include "SlamEngine.h"

class MainController
{
    public:
        MainController(int argc, char * argv[]);
        virtual ~MainController();

        int start();

        static MainController * controller;

        //Proxy functions for the GUI
        void complete();

        void save();

        void reset();

        void setPark(const bool park);

        void shutdown();

        uint64_t getMaxLag();

        //for orb-slam2
        void ORB_SLAM2();

    private:
        bool setup();
        int mainLoop();

        void loadCalibration();

        cv::Mat * depthIntrinsics;

        PangoVis * pangoVis;
        TrackerInterface * trackerInterface;
        MeshGenerator * meshGenerator;
        PlaceRecognition * placeRecognition;
        CloudSliceProcessor * cloudSliceProcessor;
        Deformation * deformation;

        SlamEngineLogReader *slamEngineRead;
        LogReader * logRead;



        boost::thread_group threads;
        std::vector<ThreadObject *> systemComponents;

        std::string directoryPath;
        std::string cameraParaterPath;
        std::string ORBvocPath;

        float mDepthMapFactor;


};

#endif /* MAINCONTROLLER_H_ */
