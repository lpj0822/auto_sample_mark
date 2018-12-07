#ifndef OPENCVMULTIPLETRACKER_H
#define OPENCVMULTIPLETRACKER_H

#include "imultipletracker.h"
#include <opencv2/tracking.hpp>

class OpencvMultipletracker : public IMultipletracker
{
public:
    OpencvMultipletracker(std::string trackingMethod);
    ~OpencvMultipletracker();

    void mutilpleTracking(const cv::Mat& preFrame, const cv::Mat& inFrame, const std::vector<TrackingObject>& objects);
    void getTrackingResult(std::vector<TrackingObject>& result);
    void initTracking();

private:

    cv::MultiTracker *trackers;
    std::string trackingMethod;
    std::vector<TrackingObject> trackingObjects;
    std::vector<std::vector<cv::Point>> trajectorys;

private:

    void init();
};

#endif // OPENCVMULTIPLETRACKER_H
