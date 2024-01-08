#ifndef PHYSICS_ANGLE_H
#define PHYSICS_ANGLE_H

namespace physics
{

constexpr float pi = 3.141592f;

// Type aliases used to distinguish angle units.
using Degree = float;
using Radian = float;

inline Radian deg2rad(Degree angle)
{
    return angle / 180.0f * pi;
}

inline Degree rad2deg(Radian angle)
{
    return angle * 180.0f / pi;
}

} // namespace physics


#endif // PHYSICS_ANGLE_H
