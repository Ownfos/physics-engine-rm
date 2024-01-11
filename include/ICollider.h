#ifndef PHYSICS_I_COLLIDER_H
#define PHYSICS_I_COLLIDER_H

#include "SFML/Graphics/Shape.hpp"
#include "Vec3.h"

namespace physics
{

enum class ColliderType
{
    ConvexPolygon,
    Circle
};

class ICollider
{
public:
    /**
     * @brief Make sure that the child class destructor gets called.
     */
    virtual ~ICollider() = default;

    /**
     * @return The maximum distance reachable from local origin.
     *         Any point outside this radius is guaranteed to be outside.
     */
    virtual float BoundaryRadius() const = 0;

    /**
     * @return The shape identifier.
     * @note This value is used to dispatch collision detection function.
     */
    virtual ColliderType Type() const = 0;

    /**
     * @param point The point we want to test.
     *              Coordinates must be expressed using
     *              this collider's local coordinate system.
     * @return True iif the point resides within this colider.
     */
    virtual bool IsPointInside(const Vec3& point) const = 0;

    /**
     * @return The surface area of this collider shape.
     */
    virtual float Area() const = 0;

    /**
     * @return The SFML representation of this collider.
     */
    virtual sf::Shape& SFMLShape() = 0;
    virtual const sf::Shape& SFMLShape() const = 0;
};

} // namespace physics

#endif // PHYSICS_I_COLLIDER_H
