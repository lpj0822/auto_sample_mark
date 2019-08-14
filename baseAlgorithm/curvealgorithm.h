#ifndef CURVEALGORITHM_H
#define CURVEALGORITHM_H

#include <opencv2/core.hpp>

#include <QList>
#include <QPoint>

class CurveAlgorithm
{
public:
    CurveAlgorithm();
    ~CurveAlgorithm();

    bool polynomialCurveFit(const QList<QPoint> &pointList, int n, cv::Mat& A);

    bool reversePolynomialCurveFit(const QList<QPoint> &pointList, int n, cv::Mat& A);
};

#endif // CURVEALGORITHM_H
