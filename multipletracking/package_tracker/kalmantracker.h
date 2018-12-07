#ifndef KALMANTRACKER_H
#define KALMANTRACKER_H

#include <vector>
#include "kalmanFilter/mykalmanfilter.h"
#include "multipletracking/trackingobject.h"

class KalmanTracker
{
public:
    KalmanTracker(TrackingObject object);
    ~KalmanTracker();

    void kalmanPrediction();//kalman预测
    void kalmanUpdate(TrackingObject object, bool isCorrect);//kalman更新
    void setSkippedFrame(size_t num);
    void addSkippedFrame(size_t num);
    size_t getSkippedFrame();
    void eraseTrace(int keepSize);
    int getTraceSize();
    std::vector<cv::Point> getTrace();
    TrackingObject getPredictObject();


private:
    std::unique_ptr<MyKalmanFilter> kalmanFilter;
    bool firstTime;
    size_t tracker_id;
    static size_t NextTrackerID;
    size_t skipped_frames;

    std::vector<cv::Rect> trace;
    TrackingObject predictObject;

    float speed;
    float dt;
    float Accel_noise_mag;

private:
    void init();
    void saveConfig();
    void loadConfig();
};

#endif // KALMANTRACKER_H
