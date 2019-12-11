#include "common_transform.h"

Rotation Transform::getRotation(float angularX, float angularY, float angularZ)
{
    float w = 0;
    float x = 0;
    float y = 0;
    float z = 0;
    w = std::cos(angularX/2) * std::cos(angularY/2) * std::cos(angularZ/2) +
            std::sin(angularX/2) * std::sin(angularY/2) * std::sin(angularZ/2);
    x = std::sin(angularX/2) * std::cos(angularY/2) * std::cos(angularZ/2) -
            std::cos(angularX/2) * std::sin(angularY/2) * std::sin(angularZ/2);
    y = std::cos(angularX/2) * std::sin(angularY/2) * std::cos(angularZ/2) +
            std::sin(angularX/2) * std::cos(angularY/2) * std::sin(angularZ/2);
    z = std::cos(angularX/2) * std::cos(angularY/2) * std::sin(angularZ/2) -
            std::sin(angularX/2) * std::sin(angularY/2) * std::cos(angularZ/2);
    return Rotation(w, x, y, z);
}
