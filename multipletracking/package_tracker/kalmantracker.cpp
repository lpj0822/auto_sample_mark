#include "kalmantracker.h"
#include <iostream>

size_t KalmanTracker::NextTrackerID = 0;

KalmanTracker::KalmanTracker(TrackingObject object): firstTime(true), predictObject(object)
{
    NextTrackerID++;
    // Here stored rect coordinates, used for next position prediction.
    init();
    std::cout << "KalmanTracker()" << std::endl;
}

KalmanTracker::~KalmanTracker()
{
    std::cout << "~KalmanTracker()" << std::endl;
}

//kalman预测
void KalmanTracker::kalmanPrediction()
{
    cv::Rect2f result = kalmanFilter->prediction();
    this->predictObject.rect = cv::Rect((int)result.x, (int)result.y, (int)result.width, (int)result.height);
    this->trace.push_back(this->predictObject.rect);
}

//kalman更新
void KalmanTracker::kalmanUpdate(TrackingObject object)
{
    cv::Rect2f rect((float)object.rect.x, (float)object.rect.y, (float)object.rect.width, (float)object.rect.height);
    cv::Rect2f result = kalmanFilter->update(rect);
    this->predictObject.rect = cv::Rect((int)result.x, (int)result.y, (int)result.width, (int)result.height);
    this->trace.push_back(this->predictObject.rect);
}

void KalmanTracker::setSkippedFrame(size_t num)
{
    this->skipped_frames = num;
}

void KalmanTracker::addSkippedFrame(size_t num)
{
    this->skipped_frames += num;
}

size_t KalmanTracker::getSkippedFrame()
{
    return this->skipped_frames;
}

void KalmanTracker::eraseTrace(int keepSize)
{
    if (keepSize <= this->getTraceSize())
        trace.erase(trace.begin(), trace.end() - keepSize);
}

int KalmanTracker::getTraceSize()
{
    return static_cast<int>(this->trace.size());
}

std::vector<cv::Point> KalmanTracker::getTrace()
{
    std::vector<cv::Point> result;
    int count = static_cast<int>(this->trace.size());
    result.clear();
    for(int loop = 0; loop < count; loop++)
    {
        int centerX = this->trace[loop].x + this->trace[loop].width / 2;
        int centerY = this->trace[loop].y + this->trace[loop].height / 2;
        result.push_back(cv::Point(centerX, centerY));
    }
    return result;
}

TrackingObject KalmanTracker::getPredictObject()
{
    return this->predictObject;
}

void KalmanTracker::init()
{
    tracker_id = NextTrackerID;
    loadConfig();
    // Every track have its own Kalman filter,
    // it user for next point position prediction.
    cv::Rect2f rect((float)predictObject.rect.x, (float)predictObject.rect.y,
                    (float)predictObject.rect.width, (float)predictObject.rect.height);
    kalmanFilter.reset(new MyKalmanFilter(rect, speed, dt, Accel_noise_mag));
    skipped_frames = 0;
    trace.clear();
}

void KalmanTracker::saveConfig()
{
    cv::FileStorage fs;
    fs.open("./config/KalmanTrack.xml", cv::FileStorage::WRITE);

    cv::write(fs, "speed", speed);
    cv::write(fs, "dt", dt);
    cv::write(fs, "Accel_noise_mag", Accel_noise_mag);

    fs.release();
}

void KalmanTracker::loadConfig()
{
    cv::FileStorage fs;
    fs.open("./config/KalmanTrack.xml", cv::FileStorage::READ);

    cv::read(fs["speed"], speed, 0.1);
    cv::read(fs["dt"], dt, 0.01f);
    cv::read(fs["Accel_noise_mag"], Accel_noise_mag, 0.5f);

    fs.release();
}
