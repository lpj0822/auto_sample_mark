#ifndef TRACKINGOBJECT_H
#define TRACKINGOBJECT_H

#include <opencv2/core.hpp>
#include <string>

class TrackingObject
{
public:
    TrackingObject();
    ~TrackingObject();

    std::string objectClass;
    cv::Rect rect;
};

#endif // TRACKINGOBJECT_H
