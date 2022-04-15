#include "core/Detector.h"

namespace RGBDSLAM
{

    std::vector<Object::Ptr> Detector::DetectObject()
    {
        std::vector<Object::Ptr> object;

        auto new_object1 = Object::CreateNewObject();
        Vec3 point1 (0.8*0.3, 0.16*0.3, 1.27*0.3);
        Vec3 dimensionn1 (0.2*0.3, 0.25*0.3, 0.42*0.3);
        double yaw1 = -0.7854;
        new_object1->id_=0;
        new_object1->dim_=dimensionn1;
        new_object1->pos_=point1;
        new_object1->yaw_=yaw1;

        auto new_object2 = Object::CreateNewObject();
        Vec3 point2 (1.2*0.3, 0.2*0.3, 5.5*0.3);
        Vec3 dimensionn2 (0.5*0.3, 4.0*0.3, 1.0*0.3);
        double yaw2 = 0;
        new_object2->id_=0;
        new_object2->dim_=dimensionn2;
        new_object2->pos_=point2;
        new_object2->yaw_=yaw2;

        object.emplace_back(new_object1);
        object.emplace_back(new_object2);

        return object;
    }

    Detector::Detector(std::string config_file_path)
    {
        if (!Config::SetParameterFile(config_file_path))
            LOG(INFO) << "No configuration file loaded.";

        if (Config::Get<std::string>("object_detector_type") == "VoteNet")
        {
            this->object_ = DetectObject();
        }
    }
}