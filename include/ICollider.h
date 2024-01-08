#ifndef PHYSICS_I_COLLIDER_H
#define PHYSICS_I_COLLIDER_H

#include "SFML/Graphics/Shape.hpp"

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
     * @return The SFML representation of this collider.
     */
    virtual sf::Shape& SFMLShape() = 0;
};

} // namespace physics

#endif // PHYSICS_I_COLLIDER_H
