#ifndef MYRECT3D_H
#define MYRECT3D_H

#include <eigen3/Eigen/Core>

class MyRect3D
{
public:
    MyRect3D();
    ~MyRect3D();

    Eigen::Vector3f center;
    Eigen::Vector3f size;
    float theta;
};

#endif // MYRECT3D_H
