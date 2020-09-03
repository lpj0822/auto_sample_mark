#include "opencvmultipletracker.h"

OpencvMultipletracker::OpencvMultipletracker(std::string trackingMethod): trackingMethod(trackingMethod)
{
    init();
}

OpencvMultipletracker::~OpencvMultipletracker()
{
    if(multiTrackers != NULL)
    {
        delete multiTrackers;
        multiTrackers = NULL;
    }
    trajectorys.clear();
}

void OpencvMultipletracker::mutilpleTracking(const cv::Mat& preFrame, const cv::Mat& inFrame, const std::vector<TrackingObject>& objects)
{
    int inputCount = static_cast<int>(objects.size());
    std::vector<cv::Rect2d> addObjects;
    std::vector<cv::Ptr<cv::Tracker> > newTrackers;
    addObjects.clear();
    if(inputCount > 0)
    {
        for(int loop = 0; loop < inputCount; loop++)
        {
            this->trackingObjects.push_back(objects[loop]);
            addObjects.push_back(objects[loop].rect);
            newTrackers.push_back(newTracker);
        }
        if(multiTrackers != NULL)
        {
            multiTrackers->add(newTrackers, preFrame, addObjects);
        }
    }
    if(multiTrackers != NULL)
    {
        multiTrackers->update(inFrame);
    }
}

void OpencvMultipletracker::getTrackingResult(std::vector<TrackingObject>& result)
{
    result.clear();
    if(multiTrackers != NULL)
    {
        std::vector<cv::Rect2d> resultObjects = multiTrackers->getObjects();
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
    if(multiTrackers != NULL)
    {
        delete multiTrackers;
        multiTrackers = NULL;
    }
    trajectorys.clear();
    trackingObjects.clear();
    createTrackerByName(trackingMethod);
    multiTrackers = new cv::MultiTracker();
}

void OpencvMultipletracker::init()
{
    multiTrackers = NULL;
    trajectorys.clear();
    trackingObjects.clear();
}

void OpencvMultipletracker::createTrackerByName(std::string name)
{
    if (name == "KCF")
        newTracker = cv::TrackerKCF::create();
    else if (name == "TLD")
        newTracker = cv::TrackerTLD::create();
    else if (name == "BOOSTING")
        newTracker = cv::TrackerBoosting::create();
    else if (name == "MEDIAN_FLOW")
        newTracker = cv::TrackerMedianFlow::create();
    else if (name == "MIL")
        newTracker = cv::TrackerMIL::create();
    else if (name == "GOTURN")
        newTracker = cv::TrackerGOTURN::create();
    else if (name == "MOSSE")
        newTracker = cv::TrackerMOSSE::create();
    else if (name == "CSRT")
        newTracker = cv::TrackerCSRT::create();
    else
        CV_Error(cv::Error::StsBadArg, "Invalid tracking algorithm name\n");
}

