#ifndef COMMON_TYPE_H_
#define COMMON_TYPE_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace algorithm_common{

// this must be at least 64 bit and signed or things will break
typedef unsigned long int TimestampUS;

//transform
typedef Eigen::Matrix<float, 6, 1> Vector6;
typedef Eigen::Quaternionf Rotation;
typedef Eigen::AngleAxisf  AngleRotation;
typedef Eigen::Vector3f  VectorRotation;
typedef Eigen::Vector3f Translation;
typedef Eigen::Matrix<float, 4, 4> Matrix;
typedef Eigen::Affine3f Affine;
}

#endif  // COMMON_TYPE_H_
