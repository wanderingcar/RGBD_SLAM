#pragma once
#ifndef RGBD_SLAM_MAP_H
#define RGBD_SLAM_MAP_H

#include "types/Frame.h"
#include "types/MapPoint.h"
#include "types/Object.h"
#include "Config.h"

namespace RGBDSLAM
{
    typedef std::unordered_map<unsigned long, MapPoint::Ptr> LandmarksType; // id and class (hash)
    typedef std::unordered_map<unsigned long, Frame::Ptr> KeyframesType;    // id and class (hash)
    typedef std::unordered_map<unsigned long, std::vector<Object::Ptr>> ObjectsType;     // id and class (hash)

    class Map
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Map> Ptr;

        Map() { min_dis_th = Config::Get<double>("keyframe_min_dist_threshold"); }

        LandmarksType GetAllMapPoints();
        KeyframesType GetAllKeyFrames();
        ObjectsType GetALLObjects();
        LandmarksType GetActiveMapPoints();
        KeyframesType GetActiveKeyFrames();
        ObjectsType GetActiveObjects();     // need to know: meaning of active
        void SetWindowSize(int window_size);

        void InsertKeyFrame(Frame::Ptr frame);
        void InsertMapPoint(MapPoint::Ptr map_point);
        void InsertObject(std::vector<Object::Ptr> object);
        void UpdateActiveMapPoint(int current_frame_id, int window_size);
        // void UpdateActiveObject(int current_frame_id, std::vector<Object::Ptr> object);
        void RemoveOldKeyframe();

        std::mutex data_mutex_;
        LandmarksType landmarks_;        // all landmarks
        LandmarksType active_landmarks_; // active landmarks
        KeyframesType keyframes_;        // all keyframes
        KeyframesType active_keyframes_; // active keyframes
        ObjectsType objects_;            // all objects
        ObjectsType active_objects_;     // active objects
        Frame::Ptr current_keyframe_;    // current keyframe
        Frame::Ptr current_frame_ = nullptr;

        std::vector<int> keyframe_id_;

        // settings --> add to configuration file
        int num_active_keyframes_ = 10;
        double min_dis_th = 0.1;
    };
}

#endif
