#ifndef PHYSICS_COLLISION_INFO_H
#define PHYSICS_COLLISION_INFO_H

#include "Vec3.h"
#include <vector>

namespace physics
{

// Forward declaration.
class Rigidbody;

// Stores set of informations calculated from a collision check
// such as collision normal vector and penetration depth.
struct CollisionInfo
{
    // Objects who collided with each other.
    Rigidbody* object1;
    Rigidbody* object2;

    // Global coordinate of points where collision occurred.
    std::vector<Vec3> contacts;

    // Normalized vector perpendicular to the collision edge.
    // This is the direction where "object2" must move
    // in order to resolve this collision.
    Vec3 normal;

    // Minimal distance required to separate two objects.
    float penetration_depth;
};

} // namespace physics

#endif // PHYSICS_COLLISION_INFO_H
