#include "curvealgorithm.h"
#include <cmath>

CurveAlgorithm::CurveAlgorithm()
{

}

CurveAlgorithm::~CurveAlgorithm()
{

}

bool CurveAlgorithm::polynomialCurveFit(const QList<QPoint> &pointList, int n, cv::Mat& A)
{
    //Number of key points
    int count = pointList.count();

    //构造矩阵X
    cv::Mat X = cv::Mat::zeros(n + 1, n + 1, CV_64FC1);
    for (int i = 0; i < n + 1; i++)
    {
        for (int j = 0; j < n + 1; j++)
        {
            for (int k = 0; k < count; k++)
            {
                X.at<double>(i, j) = X.at<double>(i, j) +
                        std::pow(pointList[k].x(), i + j);
            }
        }
    }

    //构造矩阵Y
    cv::Mat Y = cv::Mat::zeros(n + 1, 1, CV_64FC1);
    for (int i = 0; i < n + 1; i++)
    {
        for (int k = 0; k < count; k++)
        {
            Y.at<double>(i, 0) = Y.at<double>(i, 0) +
                    std::pow(pointList[k].x(), i) * pointList[k].y();
        }
    }

    A = cv::Mat::zeros(n + 1, 1, CV_64FC1);
    //求解矩阵A
    cv::solve(X, Y, A, cv::DECOMP_LU);
    return true;
}
