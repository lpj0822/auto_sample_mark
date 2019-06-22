#include "mykalmanfilter.h"
#include <iostream>

MyKalmanFilter::MyKalmanFilter(cv::Rect2f rect, float speed, float dt, float acceleration_noise_mag):
    lastResult(rect), speed(speed), deltatime(dt), acceleration_noise_mag(acceleration_noise_mag)
{
    // deltatime: time increment (lower values makes target more "massive")
    //acceleration_noise_mag:
    // We don't know acceleration, so, assume it to process noise.
    // But we can guess, the range of acceleration values thich can be achieved by tracked object.
    // Process noise. (standard deviation of acceleration: )
    // shows, woh much target can accelerate.
    //6 state variables, 4 measurements, 0 control variables
    kalman.reset(new cv::KalmanFilter(6, 4, 0, CV_32F));
    initMaxtrix();

    std::cout << "MyKalmanFilter()" << std::endl;

}

MyKalmanFilter::~MyKalmanFilter()
{
    std::cout << "~MyKalmanFilter()" << std::endl;
}

cv::Rect2f MyKalmanFilter::prediction()
{
    cv::Mat prediction = kalman->predict();
    lastResult.width = prediction.at<float>(2);
    lastResult.height = prediction.at<float>(3);
    lastResult.x = prediction.at<float>(0) - lastResult.width / 2;
    lastResult.y = prediction.at<float>(1) - lastResult.height / 2;
    return lastResult;
}

cv::Rect2f MyKalmanFilter::update(cv::Rect2f rect)
{
    //观测值
    cv::Mat measure(4, 1, CV_32FC1);
    //update using measurements
    measure.at<float>(0) = rect.x + rect.width / 2;
    measure.at<float>(1) = rect.y + rect.height / 2;
    measure.at<float>(2) = rect.width;
    measure.at<float>(3) = rect.height;

    // Correction矫正
    cv::Mat estimated = kalman->correct(measure);
    lastResult.width = estimated.at<float>(2);
    lastResult.height = estimated.at<float>(3);
    lastResult.x = estimated.at<float>(0) - lastResult.width / 2;
    lastResult.y = estimated.at<float>(1) - lastResult.height / 2;
    return lastResult;
}

void MyKalmanFilter::initMaxtrix()
{
    // Transition matrix A
    // [ 1 0 dT 0  0 0 ]
    // [ 0 1 0  dT 0 0 ]
    // [ 0 0 1  0  dT 0 ]
    // [ 0 0 0  1  0 dT ]
    // [ 0 0 0  0  1 0 ]
    // [ 0 0 0  0  0 1 ]
    kalman->transitionMatrix = (cv::Mat_<float>(6, 6) <<
                                1, 0, deltatime, 0, 0, 0,
                                0, 1, 0, deltatime, 0, 0,
                                0, 0, 1, 0, deltatime, 0,
                                0, 0, 0, 1, 0, deltatime,
                                0, 0, 0, 0, 1, 0,
                                0, 0, 0, 0, 0, 1);
    //init X vector [x,y,w,h,v_x,v_y]
    float centerX = lastResult.x + lastResult.width / 2;
    float centerY = lastResult.y + lastResult.height / 2;
    kalman->statePre.at<float>(0) = centerX; // x
    kalman->statePre.at<float>(1) = centerY; // y
    kalman->statePre.at<float>(2) = lastResult.width;
    kalman->statePre.at<float>(3) = lastResult.height;
    kalman->statePre.at<float>(4) = speed;
    kalman->statePre.at<float>(5) = speed;
    //init Z vector [z_x,z_y,z_w,z_h]
    kalman->statePost.at<float>(0) = centerX;
    kalman->statePost.at<float>(1) = centerY;
    kalman->statePost.at<float>(2) = lastResult.width;
    kalman->statePost.at<float>(3) = lastResult.height;
    //Measure matrix H
    // [ 1 0 0 0 0 0 ]
    // [ 0 1 0 0 0 0 ]
    // [ 0 0 1 0 0 0 ]
    // [ 0 0 0 1 0 0 ]
    kalman->measurementMatrix = cv::Mat::zeros(4, 6, CV_32F);
    kalman->measurementMatrix.at<float>(0) = 1.0f;
    kalman->measurementMatrix.at<float>(7) = 1.0f;
    kalman->measurementMatrix.at<float>(14) = 1.0f;
    kalman->measurementMatrix.at<float>(21) = 1.0f;
    //Wk~(0,Q) Process Noise Covariance Matrix Q
    // [ Ex   0   0     0     0    0  ]
    // [ 0    Ey  0     0     0    0  ]
    // [ 0    0   Ew    0     0    0  ]
    // [ 0    0   0     Eh    0    0  ]
    // [ 0    0   0     0     Ev_x 0  ]
    // [ 0    0   0     0     0    E_vy ]
    //cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));
    kalman->processNoiseCov = (cv::Mat_<float>(6, 6) <<
        pow(deltatime, 4.0) / 4.0, 0, pow(deltatime, 3.0) / 2.0, 0, 0, 0,
        0, pow(deltatime, 4.0) / 4.0, 0, pow(deltatime, 3.0) / 2.0, 0, 0,
        0, 0, pow(deltatime, 4.0) / 4.0, 0, pow(deltatime, 3.0) / 2.0, 0,
        0, 0, 0, pow(deltatime, 4.0) / 4.0, 0, pow(deltatime, 3.0) / 2.0,
        0, 0, 0, 0, pow(deltatime, 2.0), 0,
        0, 0, 0, 0, 0, pow(deltatime, 2.0));

    kalman->processNoiseCov *= acceleration_noise_mag;

    //Vk~(0,R) Measures Noise Covariance Maxtrix R
    // [ Ex   0   0  0 ]
    // [ 0    Ey  0  0 ]
    // [ 0    0   Ew 0 ]
    // [ 0    0   0  Eh]
    cv::setIdentity(kalman->measurementNoiseCov, cv::Scalar::all(0.5));
    //posteriori error estimate covariance matrix P
    // [ 1   0  0  0  0  0]
    // [ 0   1  0  0  0  0]
    // [ 0   0  1  0  0  0]
    // [ 0   0  0  1  0  0]
    // [ 0   0  0  0  1  0]
    // [ 0   0  0  0  0  1]
    cv::setIdentity(kalman->errorCovPost, cv::Scalar::all(0.1));
}
