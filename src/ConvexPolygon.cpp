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
        // Record edge information.
        const auto& curr = vertices[i];
        const auto& next = vertices[(i + 1) % num_vertices];
        m_edges.emplace_back(curr, next);

        // Construct SFML shape.
        m_shape.setPoint(i, {curr.x, curr.y});

        // Record the length of the farthest vertex as boundary radius.
        m_boundary_radius = std::max(m_boundary_radius, curr.Magnitude());
    }

    ValidateCounterClockwiseOrder();
}

void ConvexPolygon::ValidateCounterClockwiseOrder() const
{
    const auto num_edges = m_edges.size();
    for (int i = 0; i < m_edges.size(); ++i)
    {
        const auto& curr = m_edges[i];
        const auto& next = m_edges[(i + 1) % num_edges];
        if (curr.Tangent().Cross(next.Tangent()).z < 0.0f)
        {
            throw std::exception("the polygon was not convex");
        }
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

bool ConvexPolygon::IsPointInside(const Vec3& point) const
{
    // Key idea: since vertices are ordered counter-clockwise,
    //           an internal point should be on the left side of each edge.
    for (const auto& edge : m_edges)
    {
        if (edge.Tangent().Cross(point - edge.Start()).z < 0.0f)
        {
            return false;
        }
    }
    return true;
}

float ConvexPolygon::Area() const
{
    // Key idea: the magnitude of cross product between two vectors A and B
    //           is twice the area of triangle OAB (O is the origin).
    auto area = 0.0f;

    auto num_vertices = m_vertices.size();
    for (int i = 0; i < num_vertices; ++i)
    {
        const auto& curr = m_vertices[i];
        const auto& next = m_vertices[(i + 1) % num_vertices];

        area += curr.Cross(next).z / 2.0f;
    }

    return area;
}

sf::Shape& ConvexPolygon::SFMLShape()
{
    return m_shape;
}

const sf::Shape& ConvexPolygon::SFMLShape() const
{
    return m_shape;
}

const std::vector<Vec3>& ConvexPolygon::Vertices() const
{
    return m_vertices;
}

const std::vector<LineSegment>& ConvexPolygon::Edges() const
{
    return m_edges;
}

} // namespace physics
