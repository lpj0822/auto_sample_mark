﻿#ifndef MYKALMANFILTER_H
#define MYKALMANFILTER_H

#include <opencv2/core.hpp>
#include <opencv2/video.hpp>
#include <memory>

class MyKalmanFilter
{
public:
    MyKalmanFilter(cv::Rect2f rect, float speed, float dt = 0.2f, float acceleration_noise_mag = 0.5f);
    ~MyKalmanFilter();

    cv::Rect2f prediction();//预测
    cv::Rect2f update(cv::Rect2f rect);//更新

private:
    void initMaxtrix();

private:
    std::unique_ptr<cv::KalmanFilter> kalman;
    float speed;
    float deltatime;
    float acceleration_noise_mag;

    cv::Rect2f lastResult;//最后估计值
};

#endif // MYKALMANFILTER2_H
