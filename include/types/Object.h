// Data Structure for Semantic Objects (Landmarks)

#pragma once
#ifndef RGBD_SLAM_OBJECT_H
#define RGBD_SLAM_OBJECT_H

#include "types/Common.h"

namespace RGBDSLAM
{

    struct Frame;
    struct Feature;
    struct MapPoint;

    struct Object
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Object> Ptr;
        unsigned long id_ = 0; // ID
        bool is_outlier_ = false;
        Vec3 pos_ = Vec3::Zero(); // Position in world
        Vec3 dim_ = Vec3::Zero(); // Object Size
        double yaw_ = 0;
        std::vector<double> rgb_;
        std::mutex data_mutex_;

        unsigned long id_frame_ = 0; // first observation frame id

        Object() {}
        Object(long id, Vec3 dimension, Vec3 position, double yaw);

        Vec3 Pos();
        Vec3 Dim();
        void SetPos(const Vec3 &pos);
 
        // factory function
        static Object::Ptr CreateNewObject();
    };
}
#endif
