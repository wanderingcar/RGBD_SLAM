// Data Structure For Semantic Objects (Landmarks)

#include "types/Object.h"
#include "types/Feature.h"

namespace RGBDSLAM
{

    Object::Object(long id, Vec3 dimension, Vec3 position, double yaw) : id_(id), dim_(dimension), pos_(position), yaw_(yaw) {}

    Vec3 Object::Pos()
    {
        // share same mutex for protecting data
        std::unique_lock<std::mutex> lck(data_mutex_);
        return pos_;
    }

    Vec3 Object::Dim()
    {
        // share same mutex for protecting data
        std::unique_lock<std::mutex> lck(data_mutex_);
        return dim_;
    }

    void Object::SetPos(const Vec3 &pos)
    {
        // share same mutex for protecting data
        std::unique_lock<std::mutex> lck(data_mutex_);
        pos_ = pos;
    };

    Object::Ptr Object::CreateNewObject()
    {
        static long factory_id = 0;
        Object::Ptr new_object(new Object);
        new_object->id_ = factory_id++;
        return new_object;
    }

}
