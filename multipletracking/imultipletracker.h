#ifndef IMULTIPLETRACKER_H
#define IMULTIPLETRACKER_H

#include <vector>
#include "trackingobject.h"

class IMultipletracker
{
public:
    IMultipletracker();
    virtual ~IMultipletracker();

    virtual void mutilpleTracking(const cv::Mat& preFrame, const cv::Mat& inFrame, const std::vector<TrackingObject>& objects) = 0;
    virtual void getTrackingResult(std::vector<TrackingObject>& result) = 0;
    virtual void initTracking() = 0;
};

#endif // IMULTIPLETRACKER_H
