/*
 * This file is part of ORBFusion,modified from Kintinuous.
 */

#ifndef KINTINUOUSTRACKER_HPP_
#define KINTINUOUSTRACKER_HPP_

#include "../utils/Stopwatch.h"

#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <map>
#include <vector>
#include <limits>
#include <vector_types.h>
#include "CloudSlice.h"
#include "Resolution.h"

#include "ICPOdometry.h"
#include "RGBDOdometry.h"
#include "GroundTruthOdometry.h"
#include "../utils/ThreadMutexObject.h"
#include "PlaceRecognitionInput.h"

#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/condition_variable.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "ColorVolume.h"
#include "TSDFVolume.h"

class KintinuousTracker
{
  public:
    ThreadMutexObject<bool> tsdfRequest;
    bool tsdfAvailable;
    boost::mutex tsdfMutex;

    bool imageAvailable;
    boost::mutex imageMutex;

    bool cycledMutex;
    boost::mutex cloudMutex;
    boost::condition_variable_any cloudSignal;

    KintinuousTracker(cv::Mat * depthIntrinsics);

    virtual ~KintinuousTracker();

    void processFrame(const bool isLoop,
                      const cv::Mat mTcw,
                      const DeviceArray2D<unsigned short>& depth,
                      const DeviceArray2D<PixelRGB> & colors,
                      unsigned char * rgbImage,
                      unsigned short * depthData,
                      uint64_t timestamp,
                      bool compression,
                      uint8_t * lastCompressedDepth,
                      int depthSize,
                      uint8_t * lastCompressedImage,
                      int imageSize);

    Eigen::Vector3f getVolumeOffset() const;

    void setParked(const bool park);

    Eigen::Vector3f getLastTranslation() const;

    Eigen::Vector3f getVoxelSize() const;

    void finalise();

    Eigen::Matrix<float, 3, 3, Eigen::RowMajor> getLastRotation() const;

    std::vector<CloudSlice *> & getCloudSlices();

    void setOverlap(int overlap);

    CloudSlice * getLiveTsdf();
    CloudSlice * getLiveImage();

    void reset();

    ThreadMutexObject<uint64_t> init_utime;
    ThreadMutexObject<unsigned char *> firstRgbImage;
    ThreadMutexObject<unsigned short *> firstDepthData;

    unsigned char * lastRgbImage;
    unsigned short * lastDepthData;
    CloudSlice::Odometry lastOdometry;

    ThreadMutexObject<int> placeRecognitionId;

    static const int PR_BUFFER_SIZE = 1;
    PlaceRecognitionInput placeRecognitionBuffer[PR_BUFFER_SIZE];

    //for test


    /**
     * Loads a trajectory that is used for the motion estimate
     * instead of the ICP.
     */
    void loadTrajectory(const std::string & filename);

    class DensePose
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            DensePose(uint64_t timestamp,
                      Eigen::Matrix4f pose,
                      bool isLoopPose,
                      Eigen::Vector3f offset=Eigen::Vector3f(0,0,0)
                      )
             : timestamp(timestamp),
               pose(pose),
               isLoopPose(isLoopPose)
            {}

            DensePose()
            {}

            uint64_t timestamp;
            Eigen::Matrix4f pose;
            bool isLoopPose;
    };

    std::vector<DensePose> densePoseGraph;
    ThreadMutexObject<int> latestDensePoseId;

  private:
    Eigen::Vector3f rodrigues2(const Eigen::Matrix3f& matrix);

    /** \brief Frame counter */
    int global_time_;

    //To keep track of how far behind the backend is
    uint64_t lagTime;

    Intr intr;

    /** \brief Tsdf volume container. */
    TsdfVolume * tsdf_volume_;
    ColorVolume * color_volume_;

    Eigen::Matrix<float, 3, 3, Eigen::RowMajor> initialRotation;
    Eigen::Vector3f volumeBasis;

    /** \brief Array of dpeth pyramids. */
    std::vector<DeviceArray2D<unsigned short> > depths_curr_;

    /** \brief Array of pyramids of vertex maps for previous frame in global coordinate space. */
    std::vector<DeviceArray2D<float> > vmaps_g_prev_;
    /** \brief Array of pyramids of normal maps for previous frame in global coordinate space. */
    std::vector<DeviceArray2D<float> > nmaps_g_prev_;

    /** \brief Array of pyramids of vertex maps for current frame in current coordinate space. */
    std::vector<DeviceArray2D<float> > vmaps_curr_;
    /** \brief Array of pyramids of vertex maps for current frame in current coordinate space. */
    std::vector<DeviceArray2D<float> > nmaps_curr_;

    DeviceArray2D<uchar4> vmap_curr_color;

    /** \brief Buffer for storing scaled depth image */
    DeviceArray2D<float> depthRawScaled_;

    /** \brief Array of camera rotation matrices for each moment of time. */
    std::vector<Eigen::Matrix<float, 3, 3, Eigen::RowMajor> > rmats_;

    /** \brief Array of camera translations for each moment of time. */
    std::vector<Eigen::Vector3f>   tvecs_;

    float place_recognition_movement;

    int3 voxelWrap;
    int3 vWrapCopy;

    DeviceArray<pcl::PointXYZRGB> cloud_buffer_;
    DeviceArray<pcl::PointXYZRGB> cloud_device_;

    std::vector<CloudSlice *> sharedCloudSlices;
    CloudSlice * liveTsdf, * liveImage;

    bool cycled;
    int overlap;
    bool parked;

    DeviceArray2D<PixelRGB> modelSurface;
    DeviceArray2D<PixelRGB> modelColor;
    DeviceArray2D<unsigned short> modelDepth;
    std::vector<PixelRGB> modelHost;
    std::vector<unsigned short> modelDepthHost;

    std::map<uint64_t, Eigen::Isometry3f, std::less<int>, Eigen::aligned_allocator<std::pair<const uint64_t, Eigen::Isometry3f> > > camera_trajectory;
    uint64_t current_utime;
    uint64_t last_utime;

    ICPOdometry * icp;
    RGBDOdometry * rgbd;
    GroundTruthOdometry * groundTruth;
    OdometryProvider * odometryProvider;

    Eigen::Vector3f currentGlobalCamera;
    Eigen::Vector3f currentCameraTranslation;//for test --justin

    Eigen::Vector3f lastPlaceRecognitionTrans;
    Eigen::Matrix<float, 3, 3, Eigen::RowMajor> lastPlaceRecognitionRot;

    void outputPose(uint64_t & timestamp, Eigen::Matrix<float, 3, 3, Eigen::RowMajor> & Rcurr);

    void getImage();

    void getModelDepth();

    void allocateBuffers();

    void vWrapCopyUpdate();

    void mutexOutLiveTsdf();

    void mutexOutLiveImage();

    void mutexOutCloudBuffer(float3 & device_tcurr,
                             int3 voxelTrans,
                             PlaceRecognitionInput * placeRecognitionFrame = 0);

    void repositionCube(Eigen::Matrix<float, 3, 3, Eigen::RowMajor> & currentRotation);

    PlaceRecognitionInput * addToPlaceRecognition(int depthSize,
                                                  int imageSize,
                                                  bool compression,
                                                  unsigned short * lastCompressedDepth,
                                                  unsigned char * lastCompressedImage);


    //for test
    void mytestRotationToQuaternion();
    std::string  poseSavePath;

};

#endif /* KINTINUOUSTRACKER_HPP_ */
