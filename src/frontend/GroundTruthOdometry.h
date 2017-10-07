/*
 * This file is part of ORBFusion,modified from Kintinuous.
 */


#ifndef GROUNDTRUTHODOMETRY_H_
#define GROUNDTRUTHODOMETRY_H_

#include "OdometryProvider.h"

class GroundTruthOdometry : public OdometryProvider
{
    public:
        GroundTruthOdometry(std::vector<Eigen::Vector3f> & tvecs_,
                            std::vector<Eigen::Matrix<float, 3, 3, Eigen::RowMajor> > & rmats_,
                            std::map<uint64_t, Eigen::Isometry3f, std::less<int>, Eigen::aligned_allocator<std::pair<const uint64_t, Eigen::Isometry3f> > > & camera_trajectory,
                            uint64_t & last_utime);

        virtual ~GroundTruthOdometry();

        CloudSlice::Odometry getIncrementalTransformation(Eigen::Vector3f & trans,
                                                          Eigen::Matrix<float, 3, 3, Eigen::RowMajor> & rot,
                                                          const DeviceArray2D<unsigned short> & depth,
                                                          const DeviceArray2D<PixelRGB> & image,
                                                          uint64_t timestamp,
                                                          unsigned char * rgbImage,
                                                          unsigned short * depthData);

        Eigen::MatrixXd getCovariance();

        void reset();

        bool preRun(unsigned char * rgbImage,
                    unsigned short * depthDatam,
                    uint64_t timestamp);

    private:
        std::vector<Eigen::Vector3f> & tvecs_;
        std::vector<Eigen::Matrix<float, 3, 3, Eigen::RowMajor> > & rmats_;

        std::map<uint64_t, Eigen::Isometry3f, std::less<int>, Eigen::aligned_allocator<std::pair<const uint64_t, Eigen::Isometry3f> > > & camera_trajectory;
        uint64_t & last_utime;

};

#endif /* GROUNDTRUTHODOMETRY_H_ */
