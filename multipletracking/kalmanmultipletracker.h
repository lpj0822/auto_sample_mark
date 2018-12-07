#ifndef KALMANMULTIPLETRACKER_H
#define KALMANMULTIPLETRACKER_H

#include "package_tracker/kalmantracker.h"
#include "imultipletracker.h"

class KalmanMultipleTracker : public IMultipletracker
{
public:
    KalmanMultipleTracker();
    ~KalmanMultipleTracker();

    void mutilpleTracking(const cv::Mat& preFrame, const cv::Mat& inFrame, const std::vector<TrackingObject>& objects);
    void getTrackingResult(std::vector<TrackingObject>& result);
    void initTracking();

private:
    bool isFirstRun;//第一次运行
    std::vector<KalmanTracker*> listTrackers;//卡尔曼滤波跟踪算法

    double maxDistance;//两帧之间目标最大的移动距离
    int maxAllowedSkippedFrames;//允许目标消失的最大帧数
    int maxTraceLength;//跟踪轨迹的最大长度
    bool showOutput;//显示跟踪过程

private:

    void init();
    float computeRectDistance(const TrackingObject &object1, const TrackingObject &object2);
    void drawTrack(cv::Mat& inFrame);//绘制每个跟踪目标的轨迹

    void saveConfig();
    void loadConfig();
};

#endif // KALMANMULTIPLETRACKER_H
