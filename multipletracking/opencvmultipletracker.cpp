#include "opencvmultipletracker.h"

OpencvMultipletracker::OpencvMultipletracker(std::string trackingMethod): trackingMethod(trackingMethod)
{
    init();
}

OpencvMultipletracker::~OpencvMultipletracker()
{
    if(trackers != NULL)
    {
        delete trackers;
        trackers = NULL;
    }
    trajectorys.clear();
}

void OpencvMultipletracker::mutilpleTracking(const cv::Mat& preFrame, const cv::Mat& inFrame, const std::vector<TrackingObject>& objects)
{
    int inputCount = static_cast<int>(objects.size());
    std::vector<cv::Rect2d> addObjects;
    addObjects.clear();
    if(inputCount > 0)
    {
        for(int loop = 0; loop < inputCount; loop++)
        {
            this->trackingObjects.push_back(objects[loop]);
            addObjects.push_back(objects[loop].rect);
        }
        if(trackers != NULL)
        {
            trackers->add(preFrame, addObjects);
        }
    }
    if(trackers != NULL)
    {
        trackers->update(inFrame);
    }
}

void OpencvMultipletracker::getTrackingResult(std::vector<TrackingObject>& result)
{
    result.clear();
    if(trackers != NULL)
    {
        std::vector<cv::Rect2d> resultObjects = trackers->objects;
        int trackingCount = static_cast<int>(this->trackingObjects.size());
        for(int loop = 0; loop < trackingCount; loop++)
        {
            this->trackingObjects[loop].rect = cv::Rect((int)resultObjects[loop].x, (int)resultObjects[loop].y,
                                                       (int)resultObjects[loop].width, (int)resultObjects[loop].height);
            result.push_back(this->trackingObjects[loop]);
        }
    }
}

void OpencvMultipletracker::initTracking()
{
    if(trackers != NULL)
    {
        delete trackers;
        trackers = NULL;
    }
    trajectorys.clear();
    trackingObjects.clear();
    trackers = new cv::MultiTracker(trackingMethod);
}

void OpencvMultipletracker::init()
{
    trackers = NULL;
    trajectorys.clear();
    trackingObjects.clear();
}
