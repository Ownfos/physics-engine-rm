#ifndef PHYSICS_SPRING_H
#define PHYSICS_SPRING_H

#include "Rigidbody.h"

namespace physics
{

/**
 * @brief AnchorPoint represents one side of a spring.
 *        Since objects are dynamic, we keep the relative position.
 */
struct AnchorPoint
{
    std::shared_ptr<Rigidbody> object;
    Vec3 local_pos;

    Vec3 GlobalPosition() const;
    void ApplyImpulse(const Vec3& impulse, float delta_time);
};

/**
 * @brief Spring applies force
 * 
 */
struct Spring
{
    AnchorPoint start;
    AnchorPoint end;
    float neutral_distance;
    float coefficient;

    void ApplyImpulse(float delta_time);
};

} // namespace physics


#endif // PHYSICS_SPRING_H
