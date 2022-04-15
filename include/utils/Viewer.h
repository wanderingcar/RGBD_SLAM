//
// Created by gaoxiang on 19-5-4.
//

#ifndef MYSLAM_VIEWER_H
#define MYSLAM_VIEWER_H

#include <thread>
#include <pangolin/pangolin.h>

#include "types/Common.h"
#include "types/Frame.h"
#include "types/Map.h"

#include "core/Detector.h"


namespace RGBDSLAM
{

    class Viewer
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Viewer> Ptr;

        Viewer();

        void SetMap(Map::Ptr map) { map_ = map; }
        void SetGT(std::vector<Vec4> &q, std::vector<Vec3> &t)
        {
            gt_q = q;
            gt_t = t;
        }

        void AddCurrentFrame(Frame::Ptr current_frame);

        // void UpdateMap();

        void SpinOnce();
        void ShowResult();
        void Initialize();

        void DrawFrame(Frame::Ptr frame, const float *color);
        void DrawObject(Object::Ptr object, const float *color);

        void DrawTrajectory();
        void DrawMapPoints();

        void FollowCurrentFrame(pangolin::OpenGlRenderState &vis_camera);

        /// plot the features in current frame into an image
        cv::Mat PlotFrameImage();

        Frame::Ptr current_frame_ = nullptr;
        Map::Ptr map_ = nullptr;

        std::mutex viewer_data_mutex_;
        pangolin::View vis_display;
        pangolin::OpenGlRenderState vis_camera;

        double fx, fy, cx, cy;
        int width, height;

        std::vector<Vec4> gt_q;
        std::vector<Vec3> gt_t;
    };
}

#endif

//2D object detection
//cubeslam 볼 것. 얘는 멀티뷰 인풋 pose 정보를 사용해서 보정하는데, 우리는 depth를 실제로 앎. object를 끌어다 쓰는 것.
//indoor yolo 돌려볼 것. Cubeslam에서 쓰는 text 형태로 뽑아서 cubeslam.
//데이터셋을 standard한 form으로 맞춰달라 할 것(가능하면 pytorch)
//크몽 라벨링. 3000몇장 있는거 다 보낼 필요는 없고, 대충 4~5장 주기로(알아서 판단) 끊어서 학습 데이터 추려서 보낼 것.