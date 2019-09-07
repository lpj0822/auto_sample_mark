#include "kalmanmultipletracker.h"
#include "utility/assignmentproblemsolver.h"
#include <iostream>

cv::Scalar Colors[] = { cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255), cv::Scalar(255, 255, 0), cv::Scalar(0, 255, 255),
        cv::Scalar(255, 0, 255), cv::Scalar(255, 127, 255), cv::Scalar(127, 0, 255), cv::Scalar(127, 0, 127) };

KalmanMultipleTracker::KalmanMultipleTracker()
{
    init();
    std::cout << "KalmanMultipleTracker()" <<std::endl;
}

KalmanMultipleTracker::~KalmanMultipleTracker()
{
    initTracking();
    saveConfig();
    std::cout << "~KalmanMultipleTracker()" <<std::endl;
}

void KalmanMultipleTracker::mutilpleTracking(const cv::Mat& preFrame, const cv::Mat& inFrame, const std::vector<TrackingObject>& objects)
{
    if (objects.size() <= 0)
    {
        for (int i = 0; i < listTrackers.size(); i++)
        {
            listTrackers[i]->addSkippedFrame(1);
            if (listTrackers[i]->getSkippedFrame() > maxAllowedSkippedFrames)
            {
                delete listTrackers[i];
                listTrackers[i] = NULL;
                listTrackers.erase(listTrackers.begin() + i);
                i--;
            }
        }
        for (int i = 0; i < listTrackers.size(); i++)
        {
            listTrackers[i]->kalmanPrediction();
            if (listTrackers[i]->getTraceSize() > maxTraceLength)
            {
                listTrackers[i]->eraseTrace(maxTraceLength);
            }
        }
        return;
    }

    // If there is no tracks yet, then every point begins its own track.
    if (listTrackers.size() == 0)
    {
        // If no tracks yet,every point setup tracker
        for (int i = 0; i < objects.size(); i++)
        {
            TrackingObject object = objects[i];
            KalmanTracker* tracker = new KalmanTracker(object);
            listTrackers.push_back(tracker);
        }
        return;
    }

    int N = static_cast<int>(this->listTrackers.size());
    int M = static_cast<int>(objects.size());
    std::vector< std::vector<double> > cost(N, std::vector<double>(M));
    std::vector<int> assignment;

    float dist = 0;
    for (int i = 0; i<listTrackers.size(); i++)
    {
        for (int j = 0; j < objects.size(); j++)
        {
            dist = computeRectDistance(listTrackers[i]->getPredictObject(), objects[j]);
            cost[i][j] = static_cast<double>(dist);
        }
    }

    // Solving assignment problem (listTrackers and predictions of Kalman filter)
    AssignmentProblemSolver APS;
    APS.Solve(cost, assignment);

    // clean assignment from pairs with large distance
    // not assigned trackers
    std::vector<int> not_assigned_tracks;

    for (int i = 0; i<assignment.size(); i++)
    {
        if (assignment[i] != -1)
        {
            if (cost[i][assignment[i]] > maxDistance)
            {
                assignment[i] = -1;
                // Mark unassigned trackers, and increment skipped frames counter,
                // when skipped frames counter will be larger than threshold, track will be deleted.
                not_assigned_tracks.push_back(i);
                listTrackers[i]->addSkippedFrame(1);
            }
        }
        else
        {
            // If tracker have no assigned detect, then increment skipped frames counter.
            listTrackers[i]->addSkippedFrame(1);
        }

    }

    // If tracker didn't get detects long time, remove it.
    for (int i = 0; i<listTrackers.size(); i++)
    {
        if (listTrackers[i]->getSkippedFrame() > maxAllowedSkippedFrames)
        {
            delete listTrackers[i];
            listTrackers[i] = NULL;
            listTrackers.erase(listTrackers.begin() + i);
            assignment.erase(assignment.begin() + i);
            i--;
        }
    }

    // Search for unassigned detects
    std::vector<int> not_assigned_detections;
    std::vector<int>::iterator it;
    for (int i = 0; i < objects.size(); i++)
    {
        it = std::find(assignment.begin(), assignment.end(), i);
        if (it == assignment.end())
        {
            not_assigned_detections.push_back(i);
        }
    }

    // and start new trackers for them.
    if (not_assigned_detections.size() != 0)
    {
        for (int i = 0; i<not_assigned_detections.size(); i++)
        {
            TrackingObject object = objects[not_assigned_detections[i]];
            KalmanTracker* tracker = new KalmanTracker(object);
            listTrackers.push_back(tracker);
        }
    }

    // Update Kalman Filters state
    for (int i = 0; i<assignment.size(); i++)
    {
        if (assignment[i] != -1) // If we have assigned detect, then update using its coordinates,
        {
            TrackingObject object = objects[assignment[i]];
            listTrackers[i]->setSkippedFrame(0);
            listTrackers[i]->kalmanUpdate(object);
        }
        else // if not continue using predictions
        {
            listTrackers[i]->kalmanPrediction();
        }

        if (listTrackers[i]->getTraceSize() > maxTraceLength)
        {
            listTrackers[i]->eraseTrace(maxTraceLength);
        }
    }

    if(showOutput)
    {
        drawTrack(const_cast<cv::Mat&>(inFrame));
    }

}

void KalmanMultipleTracker::getTrackingResult(std::vector<TrackingObject>& result)
{
    int count = static_cast<int>(this->listTrackers.size());
    result.clear();
    for (int i = 0; i< count; i++)
    {
        TrackingObject object = this->listTrackers[i]->getPredictObject();
        result.push_back(object);
    }
}

void KalmanMultipleTracker::initTracking()
{
    int count = static_cast<int>(this->listTrackers.size());
    for (int i = 0; i< count; i++)
    {
        if(listTrackers[i] != NULL)
        {
            delete listTrackers[i];
            listTrackers[i] = NULL;
        }
    }
    listTrackers.clear();
    isFirstRun = true;
}

float KalmanMultipleTracker::computeRectDistance(const TrackingObject &object1, const TrackingObject &object2)
{
    float centerX1 = object1.rect.x + object1.rect.width / 2.0f;
    float centerY1 = object1.rect.y + object1.rect.height / 2.0f;
    float centerX2 = object2.rect.x + object2.rect.width / 2.0f;
    float centerY2 = object2.rect.y + object2.rect.height / 2.0f;
    float diffX = centerX1 - centerX2;
    float diffY = centerY1 - centerY2;
    float dist = std::sqrt(diffX * diffX + diffY * diffY);
    return dist;
}

//绘制每个跟踪目标的轨迹
void KalmanMultipleTracker::drawTrack(cv::Mat& inFrame)
{
    int count = (int)listTrackers.size();
    std::cout << "tracker number:"<< count << std::endl;

    for (int i = 0; i<count; i++)
    {
        if (listTrackers[i]->getTraceSize() > 1)
        {
            std::vector<cv::Point> trace = listTrackers[i]->getTrace();
            for (int j = 0; j<trace.size() - 1; j++)
            {
                cv::line(inFrame, trace[j], trace[j + 1], cv::Scalar(0,0,255), 2, cv::LINE_AA);
            }
        }
    }
}

void KalmanMultipleTracker::init()
{
    isFirstRun = true;
    listTrackers.clear();
    loadConfig();
}

void KalmanMultipleTracker::saveConfig()
{
    cv::FileStorage fs;
    fs.open("./config/KalmanMultipleTracker.xml", cv::FileStorage::WRITE);

    cv::write(fs, "maxDistance", maxDistance);
    cv::write(fs, "maxAllowedSkippedFrames", maxAllowedSkippedFrames);
    cv::write(fs, "maxTraceLength", maxTraceLength);
    cv::write(fs, "showOutput", showOutput);

    fs.release();
}

void KalmanMultipleTracker::loadConfig()
{
    cv::FileStorage fs;
    fs.open("./config/KalmanMultipleTracker.xml", cv::FileStorage::READ);

    cv::read(fs["maxDistance"], maxDistance, 60.0);
    cv::read(fs["maxAllowedSkippedFrames"], maxAllowedSkippedFrames, 20);
    cv::read(fs["maxTraceLength"], maxTraceLength, 10);
    cv::read(fs["showOutput"], showOutput, false);

    fs.release();
}
