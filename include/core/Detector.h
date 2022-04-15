#pragma once

#ifndef RGBD_SLAM_DETECTOR_H
#define RGBD_SLAM_DETECTOR_H

#include "types/Object.h"
#include "Config.h"

namespace RGBDSLAM
{
    class Detector
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Detector> Ptr;

        Detector(std::string config_file_path);
        std::vector<Object::Ptr> object_;

        std::vector<Object::Ptr> DetectObject();
    };
}

#endif