#include "ConvexPolygon.h"
#include "Circle.h"
#include <cassert>

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

        // Assuming uniform density,
        // the center of mass should be the average of all vertices.
        m_center_of_mass += curr / num_vertices;
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

bool ConvexPolygon::IsPointInside(const Vec3& local_point) const
{
    // Key idea: since vertices are ordered counter-clockwise,
    //           an internal point should be on the left side of each edge.
    for (const auto& edge : m_edges)
    {
        if (edge.Tangent().Cross(local_point - edge.Start()).z < 0.0f)
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

Vec3 ConvexPolygon::CenterOfMass() const
{
    return m_center_of_mass;
}

sf::Shape& ConvexPolygon::SFMLShape()
{
    return m_shape;
}

const sf::Shape& ConvexPolygon::SFMLShape() const
{
    return m_shape;
}

std::optional<CollisionInfo> ConvexPolygon::CheckCollision(const ICollider* other) const
{
    return other->CheckCollisionAccept(this);
}

std::optional<CollisionInfo> ConvexPolygon::CheckCollisionAccept(const Circle* other) const
{
    // Reuse Circle vs ConvexPolygon algorithm.
    auto result = other->CheckCollisionAccept(this);
    
    // Since normal vector depends on the operand order of CheckCollision(),
    // we need to flip the direction to the opposite side.
    //
    // Note that a->CheckCollision(b) and b->CheckCollision(a)
    // should have normal vectors of same magnitude and opposite direction!
    if (result)
    {
        result->normal *= -1;
    }

    return result;
}

std::optional<CollisionInfo> ConvexPolygon::CheckCollisionAccept(const ConvexPolygon* other) const
{
    
    const auto penetration_other_to_this = FindMinimumPenetration(other);
    const auto penetration_this_to_other = other->FindMinimumPenetration(this);

    // We found an axis that can separate two objects,
    // which means that there is no collision.
    if (!penetration_other_to_this || !penetration_this_to_other)
    {
        return {};
    }

    // Find the edge with minimum penetration depth.
    const auto& min_penetration = std::min(penetration_other_to_this.value(), penetration_this_to_other.value());

    // Set result.object1 as the object where min_enetration.edge came from.
    // Note that result.normal should point the direction from object1 to object2!
    auto result = CollisionInfo{
        .penetration_depth = min_penetration.depth
    };
    const ConvexPolygon* reference_obj;
    const ConvexPolygon* incident_obj;
    if (penetration_other_to_this < penetration_this_to_other)
    {
        reference_obj = this;
        incident_obj = other;
    }
    else
    {
        reference_obj = other;
        incident_obj = this;
    }

    // Since edges use local coordinate system of each object,
    // convert the start and end points to global coordinate.
    auto reference_edge = reference_obj->Transform().GlobalEdge(min_penetration.edge);
    auto incident_edge = incident_obj->FindMostParallelCollisionEdge(reference_edge.Tangent(), min_penetration.involved_vertex_index);
    auto penetrating_segment = incident_edge.Clip(reference_edge);

    // Only choose the end points inside other polygon's collider.
    for (auto end_point : {penetrating_segment.Start(), penetrating_segment.End()})
    {
        // Note: points outside a polygon have positive dot product w.r.t. the edge normal.
        if ((end_point - reference_edge.Start()).Dot(reference_edge.Normal()) < 0.0f)
        {
            result.contacts.push_back(end_point);
        }
    }
    if (incident_obj == this)
    {
        result.normal = reference_edge.Normal();
    }
    else
    {
        result.normal = -reference_edge.Normal();
    }
    
    return result;
}

const std::vector<Vec3>& ConvexPolygon::Vertices() const
{
    return m_vertices;
}

const std::vector<LineSegment>& ConvexPolygon::Edges() const
{
    return m_edges;
}

ProjectionRange ConvexPolygon::Projection(const Vec3& local_direction) const
{
    auto result = ProjectionRange{};

    auto is_first_entry = true;
    for (int i = 0; i < m_vertices.size(); ++i)
    {
        auto dot = m_vertices[i].Dot(local_direction);
        if (is_first_entry || result.min > dot)
        {
            result.min = dot;
            result.min_vertex_index = i;
        }
        if (is_first_entry || result.max < dot)
        {
            result.max = dot;
            result.max_vertex_index = i;
        }

        is_first_entry = false;
    }

    return result;
}

std::optional<Penetration> ConvexPolygon::FindMinimumPenetration(const ConvexPolygon* other) const
{
    auto result = std::optional<Penetration>{};

    // From now on, everything will be calculated under polygon1's coordinate system.
    for (const auto& edge : Edges())
    {
        // Projection of polygon1 onto the normal vector,
        // assuming that the polygon is placed on the origin.
        auto normal = edge.Normal();
        const auto projection1 = Projection(normal);


        // This is the projection of relative displacement (p1 to p2) on the global normal vector.
        // Since colliders uses their own local coordinate system,
        // we need to manually adjust the difference in the objects' positions.
        const auto global_normal = Transform().GlobalDirection(normal);
        const auto offset_polygon2 = (other->Transform().Position() - Transform().Position()).Dot(global_normal);

        // Projection of polygon2 onto the normal vector,
        // assuming that the polygon is placed on the origin.
        const auto local_normal = other->Transform().LocalDirection(global_normal);
        auto projection2 = other->Projection(local_normal);
        projection2.min += offset_polygon2;
        projection2.max += offset_polygon2;

        // A separating axis implies no collision!
        if (projection1.IsSeparated(projection2))
        {
            return {};
        }

        const auto overlap = projection1.max - projection2.min;
        if (!result.has_value() || result->depth > overlap)
        {
            result.emplace(edge, overlap, projection2.min_vertex_index);
        }
    }

    // If the algorithm is valid, penetration depth should always be positive.
    assert(result->depth > 0.0f);

    return result;
}

LineSegment ConvexPolygon::FindMostParallelCollisionEdge(const Vec3& global_dir, int involved_vertex_index) const
{
    // Get two edges that contain the vertex involved in collision.
    // Note: Given vertex index x, the edges we need is edges[x] and edges[x - 1].
    //       To prevent x - 1 from going negative, add and modulo edges.size() was used.
    const auto& edges = Edges();
    const auto edge1 = Transform().GlobalEdge(edges[involved_vertex_index]);
    const auto edge2 = Transform().GlobalEdge(edges[(involved_vertex_index + edges.size() - 1) % edges.size()]);

    // Choose the one with tangent direction more similar to the given direction vector.
    // Note: std::abs() was used to handle edge directions parallel but opposite.
    if (std::abs(edge1.Tangent().Dot(global_dir))
        > std::abs(edge2.Tangent().Dot(global_dir)))
    {
        return edge1;
    }
    else
    {
        return edge2;
    }
}

} // namespace physics
