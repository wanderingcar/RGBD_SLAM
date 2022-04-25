#include "core/Detector.h"


namespace RGBDSLAM
{

    Detector::Detector(std::string config_file_path)
    {
        if (!Config::SetParameterFile(config_file_path))
            LOG(INFO) << "No configuration file loaded.";

        if (Config::Get<std::string>("object_detector_type") == "VoteNet")
        {
            //this->object_ = DetectObject();
        }
    }

    std::vector<std::string> split(std::string input, char delimiter)
    {
        std::vector<std::string> answer;
        std::stringstream ss(input);
        std::string temp;

        while (getline(ss, temp, delimiter))
        {
            answer.push_back(temp);
        }

        return answer;
    }

    void Detector::DetectEdge(std::string file_number)
    {
        std::string file_path = "/home/cadit/src/copy/RGBD_SLAM/DATA_FOLDER/keyframe_set_annotated/obj_train_data/";
        cv::Mat raw_img = cv::imread(file_path + file_number + ".png", 1);
        if(raw_img.data == NULL )
        {
            std::cerr << "Error, image could not be loaded. Please, check its path \n"<<file_path << std::endl;
            abort();
        }

        bool use_LSD_algorithm=true;
        bool save_to_imgs=true;
        bool save_to_txts=true;

        int numOfOctave_ = 1;
        float Octave_ratio = 2.0;  

        line_lbd_detect* line_lbd_ptr = new line_lbd_detect(numOfOctave_,Octave_ratio);
        line_lbd_ptr->use_LSD = use_LSD_algorithm;
        line_lbd_ptr->line_length_thres = 15;  // remove short edges

        // using my line detector class, could select LSD or edline.
        cv::Mat out_edges;
        std::vector< KeyLine> keylines_raw,keylines_out;
        line_lbd_ptr->detect_raw_lines(raw_img,keylines_raw);
        line_lbd_ptr->filter_lines(keylines_raw,keylines_out);  // remove short lines

        // show image
        if( raw_img.channels() == 1 )
            cvtColor( raw_img, raw_img, cv::COLOR_GRAY2BGR );
        cv::Mat raw_img_cp;
        drawKeylines(raw_img, keylines_out, raw_img_cp, cv::Scalar( 0, 150, 0 ),2); // B G R
        //imshow( "Line detector", raw_img_cp );
        //cv::waitKey();

        if (save_to_imgs)
        {
            std::string img_save_name = file_path+ "saved_edges" + file_number + ".jpg";
            cv::imwrite(img_save_name,raw_img_cp);
        }
        
        if (save_to_txts)
        {
            std::string txt_save_name = file_path+"saved_edges"+file_number+".txt";
            std::ofstream resultsFile;
            resultsFile.open(txt_save_name);
            for (int j=0;j<keylines_out.size();j++)
            {
            resultsFile <<keylines_out[j].startPointX <<"\t" <<keylines_out[j].startPointY  <<"\t"
                    <<keylines_out[j].endPointX   <<"\t" <<keylines_out[j].endPointY    <<std::endl;
            }
            resultsFile.close();
        }
    }

    void Detector::DetectObject(Frame::Ptr keyframe)
    {
        Eigen::MatrixXd param_pass(10,5);
        param_pass.setZero();

        std::string file_path = "/home/cadit/src/copy/RGBD_SLAM/DATA_FOLDER/keyframe_set_annotated/obj_train_data/";
        std::string file_num = std::to_string(keyframe->keyframe_id_);

        std::ifstream filetxt((file_path + file_num + ".txt").c_str());

        std::string line;
        
        int i=0;
        while (filetxt)
        {
            getline(filetxt, line);
            if (!line.empty())
            {
                std::vector<double> one_object;
                std::vector<std::string> buffer = split(line,' '); // 1칸 당겨서 받고 맨 뒤에 확률 넣는 것으로 바꿔야 함
                
                param_pass(i,0) = std::round(stod(buffer[1])*848);
                param_pass(i,1) = std::round(stod(buffer[2])*480);
                param_pass(i,2) = std::round(stod(buffer[3])*848);
                param_pass(i,3) = std::round(stod(buffer[4])*480);
                param_pass(i,4) = 1.0;
                i++;
            }
        }

        Eigen::MatrixXd obj_bbox_coors = param_pass.topRows(i);
        filetxt.close();

        cv::Mat rgb_img = cv::imread(file_path + file_num + ".png", 1);

        DetectEdge(file_num);

        Eigen::MatrixXd all_lines_raw(100, 4);
        read_all_number_txt(file_path + "saved_edges" + file_num + ".txt", all_lines_raw); // detect line (raw)


        detect_3d_cuboid detect_cuboid_obj;
        detect_cuboid_obj.set_calibration(keyframe->K_);

        // std::cout << detect_cuboid_obj.cam_pose.Kalib << std::endl;


        std::vector<ObjectSet> all_object_cuboids;

        SE3 tcw;
        Sophus::Matrix4d TCW;
        tcw=keyframe->Pose().inverse();


        Sophus::Matrix4d T;
        T << 1, 0, 0, 0,
             0, 0, 1, 0,
             0,-1, 0, 0.37,
             0, 0, 0, 1 ;

        TCW=T*tcw.matrix().template cast<double>();

        detect_cuboid_obj.detect_cuboid(rgb_img, TCW, obj_bbox_coors, all_lines_raw, all_object_cuboids, file_num);

        Eigen::Matrix3d T_;
        T_ << 1, 0, 0,
             0, 0,-1,
             0, 1, 0;
        for (auto& j : all_object_cuboids)
        {
            if(j.size()!=0)
            {
                Eigen::Matrix<double, 4, 8> temp;
                temp<<j[0]->box_corners_3d_world, Eigen::Matrix<double, 1, 8>::Ones(8);

                temp=T.inverse()*temp;
                for(int i=0;i<3;i++)
                    j[0]->box_corners_3d_world.row(i)=temp.row(i);
                    
                j[0]->pos=j[0]->box_corners_3d_world.rowwise().mean();

                Object::Ptr one_object = Object::CreateNewObject();
                one_object->pos_=j[0]->pos;
                //one_object->dim_=keyframe->camera2world(j[0]->scale);
                one_object->dim_=j[0]->scale;
                one_object->yaw_=j[0]->rotY;     
                keyframe->object_.push_back(one_object);
            }
        }

    }
}