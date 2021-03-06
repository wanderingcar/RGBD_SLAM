#include "utils/Viewer.h"
#include "types/Feature.h"
#include "types/Frame.h"

#include <pangolin/pangolin.h>
#include <opencv2/opencv.hpp>
#include <ctime>

#include "Config.h"

namespace RGBDSLAM
{

    Viewer::Viewer()
    {

        fx = Config::Get<double>("camera.fx");
        fy = Config::Get<double>("camera.fy");
        cx = Config::Get<double>("camera.cx");
        cy = Config::Get<double>("camera.cy");
        height = Config::Get<int>("height");
        width = Config::Get<int>("width");
    }

    void Viewer::AddCurrentFrame(Frame::Ptr current_frame)
    {
        std::unique_lock<std::mutex> lck(viewer_data_mutex_);
        current_frame_ = current_frame;
    }

    void Viewer::Initialize()
    {
        pangolin::CreateWindowAndBind("MySLAM", 1024, 768);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        pangolin::OpenGlRenderState vis_camera_(pangolin::ProjectionMatrix(1024, 768, 400, 400, 512, 384, 0.1, 1000),
                                                pangolin::ModelViewLookAt(0, -5, -10, 0, 0, 0, 0.0, -1.0, 0.0));

        vis_camera = vis_camera_;

        vis_display = pangolin::CreateDisplay().SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f).SetHandler(new pangolin::Handler3D(vis_camera));
    }

    cv::Mat Viewer::PlotFrameImage()
    {
        cv::Mat img_out;
        // cv::cvtColor(current_frame_->rgb_, img_out, CV_GRAY2BGR);
        for (size_t i = 0; i < current_frame_->features_.size(); ++i)
        {
            if (current_frame_->features_[i]->map_point_.lock())
            {
                auto feat = current_frame_->features_[i];

                cv::circle(img_out, feat->position_.pt, 2, cv::Scalar(0, 250, 0), 2);
            }
        }
        return img_out;
    }

    void Viewer::FollowCurrentFrame(pangolin::OpenGlRenderState &vis_camera)
    {
        SE3 Twc = current_frame_->Pose().inverse();
        pangolin::OpenGlMatrix m(Twc.matrix());
        vis_camera.Follow(m, true);
    }

    void Viewer::DrawFrame(Frame::Ptr frame, const float *color)
    {
        SE3 Twc = frame->Pose().inverse();
        const float sz = 0.3;
        const int line_width = 2.0;

        glPushMatrix();

        Sophus::Matrix4f m = Twc.matrix().template cast<float>();
        Sophus::Matrix4d twc = Twc.matrix().template cast<double>();
        glMultMatrixf((GLfloat *)m.data());

        if (color == nullptr)
        {
            glColor3f(1, 0, 0);
        }
        else
            glColor3f(color[0], color[1], color[2]);

        glLineWidth(line_width);
        glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

        glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

        glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

        glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);

        glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

        glEnd();
        glPopMatrix();
    }


    // void Viewer::DrawCuboids(cuboid* cuboid_, const float *color)
    // {
    //     const float sz = 0.3;
    //     const int line_width = 2.0;

    //     Eigen::MatrixXd all_edge_pt_ids, front_edge_pt_ids;
    //     all_edge_pt_ids.resize(8, 2); // draw 8 edges except front face
	//     all_edge_pt_ids << 2, 3, 3, 4, 4, 1, 3, 7, 4, 8, 6, 7, 7, 8, 8, 5;
    //     all_edge_pt_ids.array() -= 1;
    //     front_edge_pt_ids.resize(4,2);
    //     front_edge_pt_ids << 1, 2, 2, 6, 6, 5, 5, 1;
    //     front_edge_pt_ids.array() -= 1;
        
    //     Eigen::MatrixXd cube_corners;
    //     cube_corners=cuboid_->box_corners_3d_world;
    //     std::cout<<cube_corners<<std::endl;
    //     glPushMatrix();
        
    //     if (color == nullptr)
    //     {
    //         glColor3f(1, 0, 0);
    //     }
    //     else
    //         glColor3f(color[0], color[1], color[2]);
        
    //     glLineWidth(line_width);
    //     glBegin(GL_LINES);

    //     for (int line_id = 0; line_id < all_edge_pt_ids.rows(); line_id++)
	// 	{
	// 		glVertex3f(cube_corners(0, all_edge_pt_ids(line_id, 0)), cube_corners(1, all_edge_pt_ids(line_id, 0)), cube_corners(2, all_edge_pt_ids(line_id, 0)));
	// 		glVertex3f(cube_corners(0, all_edge_pt_ids(line_id, 1)), cube_corners(1, all_edge_pt_ids(line_id, 1)), cube_corners(2, all_edge_pt_ids(line_id, 1)));
	// 	}
    //     for (int line_id = 0; line_id < front_edge_pt_ids.rows(); line_id++)
	// 	{
	// 		glVertex3f(cube_corners(0, front_edge_pt_ids(line_id, 0)), cube_corners(1, front_edge_pt_ids(line_id, 0)), cube_corners(2, front_edge_pt_ids(line_id, 0)));
	// 		glVertex3f(cube_corners(0, front_edge_pt_ids(line_id, 1)), cube_corners(1, front_edge_pt_ids(line_id, 1)), cube_corners(2, front_edge_pt_ids(line_id, 1)));
	// 	}
	// 	glEnd();
    //     glPopMatrix();
    // }

    //Draw object 3D bounding box
    void Viewer::DrawObject(Object::Ptr object, const float *color)
    {
        const float sz = 0.3;
        const int line_width = 2.0;

        glPushMatrix();

        if (color == nullptr)
        {
            glColor3f(1, 0, 0);
        }
        else
            glColor3f(color[0], color[1], color[2]);

        glLineWidth(line_width);
        glBegin(GL_LINES);

        std::vector<Vec3> p;
        double cos_=std::cos(object->yaw_);
        double sin_=std::sin(object->yaw_);

        
        p.emplace_back(Vec3(sz*object->pos_[0]+sz*object->dim_[0]/2.0f*cos_+sz*object->dim_[1]/2.0f*sin_,
                            sz*object->pos_[1]+sz*object->dim_[2]/2.0f,
                             sz*object->pos_[2]+sz*object->dim_[1]/2.0f*cos_-sz*object->dim_[0]/2.0f*sin_));
        p.emplace_back(Vec3(sz*object->pos_[0]-sz*object->dim_[0]/2.0f*cos_+sz*object->dim_[1]/2.0f*sin_,
                            sz*object->pos_[1]+sz*object->dim_[2]/2.0f,
                             sz*object->pos_[2]+sz*object->dim_[1]/2.0f*cos_+sz*object->dim_[0]/2.0f*sin_));
        p.emplace_back(Vec3(sz*object->pos_[0]-sz*object->dim_[0]/2.0f*cos_-sz*object->dim_[1]/2.0f*sin_,
                            sz*object->pos_[1]+sz*object->dim_[2]/2.0f,
                             sz*object->pos_[2]-sz*object->dim_[1]/2.0f*cos_+sz*object->dim_[0]/2.0f*sin_));
        p.emplace_back(Vec3(sz*object->pos_[0]+sz*object->dim_[0]/2.0f*cos_-sz*object->dim_[1]/2.0f*sin_,
                            sz*object->pos_[1]+sz*object->dim_[2]/2.0f,
                             sz*object->pos_[2]-sz*object->dim_[1]/2.0f*cos_-sz*object->dim_[0]/2.0f*sin_));
        p.emplace_back(Vec3(sz*object->pos_[0]+sz*object->dim_[0]/2.0f*cos_+sz*object->dim_[1]/2.0f*sin_,
                            sz*object->pos_[1]-sz*object->dim_[2]/2.0f,
                             sz*object->pos_[2]+sz*object->dim_[1]/2.0f*cos_-sz*object->dim_[0]/2.0f*sin_));
        p.emplace_back(Vec3(sz*object->pos_[0]-sz*object->dim_[0]/2.0f*cos_+sz*object->dim_[1]/2.0f*sin_,
                            sz*object->pos_[1]-sz*object->dim_[2]/2.0f,
                             sz*object->pos_[2]+sz*object->dim_[1]/2.0f*cos_+sz*object->dim_[0]/2.0f*sin_));
        p.emplace_back(Vec3(sz*object->pos_[0]-sz*object->dim_[0]/2.0f*cos_-sz*object->dim_[1]/2.0f*sin_,
                            sz*object->pos_[1]-sz*object->dim_[2]/2.0f,
                             sz*object->pos_[2]-sz*object->dim_[1]/2.0f*cos_+sz*object->dim_[0]/2.0f*sin_));
        p.emplace_back(Vec3(sz*object->pos_[0]+sz*object->dim_[0]/2.0f*cos_-sz*object->dim_[1]/2.0f*sin_,
                            sz*object->pos_[1]-sz*object->dim_[2]/2.0f,
                             sz*object->pos_[2]-sz*object->dim_[1]/2.0f*cos_-sz*object->dim_[0]/2.0f*sin_));


        int i=0;

        for(i=0;i<8;i++)
        {
            glVertex3f(p[i][0],p[i][1],p[i][2]);
        }

        glVertex3f(p[0][0],p[0][1],p[0][2]);
        glVertex3f(p[3][0],p[3][1],p[3][2]);

        glVertex3f(p[1][0],p[1][1],p[1][2]);
        glVertex3f(p[2][0],p[2][1],p[2][2]);

        glVertex3f(p[4][0],p[4][1],p[4][2]);
        glVertex3f(p[7][0],p[7][1],p[7][2]);

        glVertex3f(p[5][0],p[5][1],p[5][2]);
        glVertex3f(p[6][0],p[6][1],p[6][2]);

        for(i=0;i<4;i++)
        {
            glVertex3f(p[i][0],p[i][1],p[i][2]);
            glVertex3f(p[i+4][0],p[i+4][1],p[i+4][2]);
        }

        glEnd();
        glPopMatrix();
    }

    void Viewer::DrawTrajectory()
    {
        const int line_width = 5.0;

        std::vector<Vec3> traj;

        for (int i = 0; i < (int)map_->keyframe_id_.size(); i++)
        {
            int id = map_->keyframe_id_[i];
            auto item = map_->keyframes_.find(id);
            if (item != map_->keyframes_.end())
            {
                SE3 Twc = item->second->Pose().inverse();
                Vec3 Translation = Twc.translation();
                traj.emplace_back(Vec3(Translation[0], Translation[1], Translation[2]));
            }
        }

        glColor3f(0, 1, 0);
        glLineWidth(line_width);
        glBegin(GL_LINES);

        for (int i = 0; i < (int)map_->keyframe_id_.size() - 1; i++)
        {
            Vec3 position_i = traj[i];
            Vec3 position_j = traj[i + 1];
            glVertex3f(position_i[0], position_i[1], position_i[2]);
            glVertex3f(position_j[0], position_j[1], position_j[2]);
        }

        glEnd();

        if (gt_t.size() > 0)
        {

            glColor3f(1, 0, 0);
            glLineWidth(line_width);
            glBegin(GL_LINES);

            for (int i = 0; i < current_frame_->id_ - 1; i++)
            {
                Vec3 position_i(gt_t[i][0], gt_t[i][1], gt_t[i][2]);
                Vec3 position_j(gt_t[i + 1][0], gt_t[i + 1][1], gt_t[i + 1][2]);
                glVertex3f(position_i[0], position_i[1], position_i[2]);
                glVertex3f(position_j[0], position_j[1], position_j[2]);
            }

            glEnd();
        }
    }

    void Viewer::DrawMapPoints()
    {

        const float red[3] = {1.0, 0, 0};

        // LOG(INFO) <<"The number of keyframes: "<<map_->keyframes_.size();

        // for (auto &kf : map_->keyframes_)
        // {
        //     DrawFrame(kf.second, red);
        // }

        glPointSize(2);
        glBegin(GL_POINTS);
        for (auto &landmark : map_->landmarks_)
        {
            auto pos = landmark.second->Pos();
            auto rgb = landmark.second->rgb_;
            glColor3f(rgb[0] / 255., rgb[1] / 255., rgb[2] / 255.);
            glVertex3d(pos[0], pos[1], pos[2]);
        }
        glEnd();

        glPointSize(2);
        glBegin(GL_POINTS);
        for (auto &landmark : map_->active_landmarks_)
        {
            auto pos = landmark.second->Pos();
            glColor3f(red[0], red[1], red[2]);
            glVertex3d(pos[0], pos[1], pos[2]);
        }
        glEnd();
    }

    void Viewer::SpinOnce()
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        vis_display.Activate(vis_camera);

        // Object::Ptr object = Object::CreateNewObject();
        // object->pos_=Vec3(0.8, 0.16, 1.27);
        // object->dim_=Vec3(0.25, 0.2, 0.42);
        // object->yaw_=-0.7854;


        const float green[3] = {0.0, 1.0, 0.0};
        const float black[3] = {0.0, 0.0, 0.0};
        std::unique_lock<std::mutex> lock(viewer_data_mutex_);

        const float red[3] = {1.0, 0, 0};
       
        // DrawObject(object, red);

        for (auto &kf : map_->active_keyframes_)
        {
            DrawFrame(kf.second, red);
            // for(int obj_id=0;obj_id<kf.second->frames_cuboids.size();obj_id++)
            //     DrawCuboids(kf.second->frames_cuboids[obj_id][0],red);
            for(auto& obj : kf.second->object_)
                DrawObject(obj,red);
        }

        DrawTrajectory();
        // for (auto &kf : map_->keyframes_)
        // {
        //     DrawFrame(kf.second, black);
        // }

        DrawFrame(current_frame_, green);
        //FollowCurrentFrame(vis_camera);
        cv::Mat imSmall;
        cv::resize(current_frame_->rgb_, imSmall, cv::Size(current_frame_->rgb_.cols / 2.0, current_frame_->rgb_.rows / 2.0));
        // cv::imwrite("/home/cadit/test.jpg", imSmall);
        // pangolin::LoadImage("/home/cadit/test.jpg");
        cv::imshow("image", imSmall);
        cv::waitKey(1);

        DrawMapPoints();
        pangolin::FinishFrame();
    }

    void Viewer::ShowResult()
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        vis_display.Activate(vis_camera);

        const float green[3] = {0.0, 1.0, 0.0};
        std::unique_lock<std::mutex> lock(viewer_data_mutex_);
        const float red[3] = {1.0, 0, 0};
        const float blue[3] = {0.0, 0, 1.0};

        for (auto &kf : map_->keyframes_)
        {
            DrawFrame(kf.second, blue);
        }

        DrawFrame(current_frame_, green);
        // FollowCurrentFrame(vis_camera);
        //  cv::imshow("image", current_frame_->rgb_);

        DrawTrajectory();

        DrawMapPoints();
        pangolin::FinishFrame();
    }

}