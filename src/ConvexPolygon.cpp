#include "ConvexPolygon.h"

namespace physics
{

ConvexPolygon::ConvexPolygon(const std::vector<Vec3>& vertices)
    : m_vertices(vertices)
{
    const auto num_vertices = vertices.size();
    m_shape.setPointCount(num_vertices);
    for (int i = 0; i < num_vertices; ++i)
    {
        const auto& v = vertices[i];

        // Construct SFML shape.
        m_shape.setPoint(i, {v.x, v.y});

        // Record the length of the farthest vertex as boundary radius.
        m_boundary_radius = std::max(m_boundary_radius, v.Magnitude());
    }
}

float ConvexPolygon::BoundaryRadius() const
{
    return m_boundary_radius;
}

ColliderType ConvexPolygon::Type() const
{
    return ColliderType::ConvexPolygon;
}

sf::Shape& ConvexPolygon::SFMLShape()
{
    return m_shape;
}

const sf::Shape& ConvexPolygon::SFMLShape() const
{
    return m_shape;
}

} // namespace physics
