#pragma once

#ifndef RGBD_SLAM_DETECTOR_H
#define RGBD_SLAM_DETECTOR_H

#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
// #include <Eigen>

#include "detect_3d_cuboid/detect_3d_cuboid.h"
#include "line_lbd/line_lbd_allclass.h"

#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include <Eigen/Dense>
#include <Eigen/Core>

#include "types/Object.h"
#include "types/Frame.h"
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

        void DetectEdge(std::string file_number);

        void DetectObject(Frame::Ptr keyframe);
        
    };
}

#endif