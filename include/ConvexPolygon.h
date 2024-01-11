#ifndef PHYSICS_CONVEX_POLYGON_H
#define PHYSICS_CONVEX_POLYGON_H

#include "ICollider.h"
#include "LineSegment.h"
#include "SFML/Graphics/ConvexShape.hpp"
#include <vector>

namespace physics
{

class ConvexPolygon : public ICollider
{
public:
    /**
     * @note The order of @p vertices must be counter-clockwise!
     *       If not, an exception will be thrown.
     */
    ConvexPolygon(const std::vector<Vec3>& vertices);

    virtual float BoundaryRadius() const override;
    virtual ColliderType Type() const override;
    virtual bool IsPointInside(const Vec3& point) const override;
    virtual float Area() const override;
    virtual sf::Shape& SFMLShape() override;
    virtual const sf::Shape& SFMLShape() const override;

    const std::vector<Vec3>& Vertices() const;
    const std::vector<LineSegment>& Edges() const;
    
private:
    /**
     * @brief Throw exception if the order of vertices is not counter-clockwise.
     */
    void ValidateCounterClockwiseOrder() const;

    std::vector<Vec3> m_vertices;
    std::vector<LineSegment> m_edges;

    // SFML representation.
    sf::ConvexShape m_shape;

    float m_boundary_radius;
};

} // namespace physics

#endif // PHYSICS_CONVEX_POLYGON_H
