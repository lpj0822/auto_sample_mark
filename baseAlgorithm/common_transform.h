#ifndef COMMON_TRANSFORM_H
#define COMMON_TRANSFORM_H

#include "common_type.h"

using namespace algorithm_common;

class Transform
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Transform()
    {
        rotation_.setIdentity();
        translation_.setZero();
    }

    Transform(const Translation& translation, const Rotation& rotation)
        :rotation_(rotation), translation_(translation) {}

    const Rotation& rotation() const { return rotation_; }

    VectorRotation vectorRation()
    {
        AngleRotation angleAxis(rotation_);
        VectorRotation vector = angleAxis.angle() * angleAxis.axis();
        return vector;
    }

    void setTransform(const Matrix &matrix)
    {
        rotation_ = Rotation( matrix.topLeftCorner<3, 3>());
        translation_ = matrix.topRightCorner<3, 1>();
    }

    const Translation& translation() const { return translation_; }

    Matrix matrix() const
    {
        Matrix matrix;
        matrix.setIdentity();
        matrix.topLeftCorner<3, 3>() = rotation_.matrix();
        matrix.topRightCorner<3, 1>() = translation_;
        return matrix;
    }

    Transform inverse() const
    {
        const Rotation rotation_inverted(rotation_.w(), -rotation_.x(),
                                         -rotation_.y(), -rotation_.z());
        return Transform(-(rotation_inverted * translation_), rotation_inverted);
    }

    Transform operator*(const Transform& rhs) const
    {
        return Transform(translation_ + rotation_ * rhs.translation(),
                         rotation_ * rhs.rotation());
    }

    Transform exp(const Vector6& vector)
    {
        constexpr float kEpsilon = 1e-8;
        const float norm = vector.tail<3>().norm();
        if (norm < kEpsilon) {
            return Transform(vector.head<3>(), Rotation::Identity());
        } else {
            return Transform(vector.head<3>(), Rotation(Eigen::AngleAxisf(
                                                            norm, vector.tail<3>() / norm)));
        }
    }

    Vector6 log() const
    {
        Eigen::AngleAxisf angle_axis(rotation_);
        return (Vector6() << translation_, angle_axis.angle() * angle_axis.axis()).finished();
    }

    Rotation getRotation(float angularX, float angularY, float angularZ);

private:
    Rotation rotation_;
    Translation translation_;
};

#endif // COMMON_TRANSFORM_H
