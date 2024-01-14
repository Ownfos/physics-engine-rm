#ifndef PHYSICS_I_COLLIDER_H
#define PHYSICS_I_COLLIDER_H

#include "SFML/Graphics/Shape.hpp"
#include "Transform.h"
#include <optional>

namespace physics
{

enum class ColliderType
{
    ConvexPolygon,
    Circle
};

struct CollisionInfo
{
    // Global coordinate of points where collision occurred.
    std::vector<Vec3> contacts;

    // Normalized vector perpendicular to the collision edge.
    // This is the direction where "this" object must move
    // in order to resolve this collision.
    //
    // @see ICollider::CheckCollision() for how the direction is decided.
    Vec3 normal;

    // Minimal distance required to separate two objects.
    float penetration_depth;
};

// Forward declarations for ICollider::CheckCollisionAccept().
class Circle;
class ConvexPolygon;

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
    virtual bool IsPointInside(const Vec3& local_point) const = 0;

    /**
     * @return The surface area of this collider shape.
     */
    virtual float Area() const = 0;

    /**
     * @return The central point, assuming that density is uniform.
     */
    virtual Vec3 CenterOfMass() const = 0;

    /**
     * @return The SFML representation of this collider.
     */
    virtual sf::Shape& SFMLShape() = 0;
    virtual const sf::Shape& SFMLShape() const = 0;

    /**
     * @brief Return collision information if any.
     * 
     * @note Collision detection depends on the collider type of both objects.
     *       The double dispatch is implemented in a form of visitor pattern.
     *       CheckCollision() corresponds to Accept(Visitor*),
     *       while CheckCollisionAccept() corresponds to Visit().
     * 
     * @note 'this' and 'other' is swapped when we call CheckCollisionAccept().
     *       Since CheckCollisionAccept() gives a normal vector where its 'this' must move, 
     *       the result, in this context, becomes the direction where 'other' must move.
     *       Quite confusing...
     * 
     *       - example scenario -
     *       step 1) a->CheckCollision(b)
     *               - this: a
     *               - other: b
     *               - result.normal: direction from 'this' to 'other' (a -> b)
     *       step 2) b->CheckCollisionAccept(a)  | 'this': b, 'other': a
     *               - this: b
     *               - other: a
     *               - result.normal: direction from 'other' to 'this' (a -> b)
     * 
     * @see ICollider::CheckCollisionAccept()
     */
    virtual std::optional<CollisionInfo> CheckCollision(const ICollider* other) const = 0;

    /**
     * @brief These functions correspond to the Visit() method of visitor pattern.
     *        Whenever a user calls a->CheckCollision(b),
     *        a will invoke b->CheckCollisionAccept(this) to perform double dispatch.
     *        Then b, which is 'this' in current context, will run the actual
     *        collision detection algorithm such as 'Polygon vs Circle'.
     * 
     * @note The argument collider @p other is the caller of CheckCollision().
     * 
     * @see ICollider::CheckCollision()
     */
    virtual std::optional<CollisionInfo> CheckCollisionAccept(const Circle* other) const = 0;
    virtual std::optional<CollisionInfo> CheckCollisionAccept(const ConvexPolygon* other) const = 0;

    inline physics::Transform& Transform()
    {
        return m_transform;
    }

    inline const physics::Transform& Transform() const
    {
        return m_transform;
    }

protected:

    physics::Transform m_transform;
};

} // namespace physics

#endif // PHYSICS_I_COLLIDER_H
