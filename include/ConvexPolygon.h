#ifndef PHYSICS_CONVEX_POLYGON_H
#define PHYSICS_CONVEX_POLYGON_H

#include "ICollider.h"
#include "LineSegment.h"
#include "SFML/Graphics/ConvexShape.hpp"
#include <vector>

namespace physics
{

// Used for SAT algorithm to store the minimum and maximum
// dot product between all vertices and a single direction vector.
// @see FindMinimumPenetration() defined in Rigidbody.cpp
struct ProjectionRange
{
    float min;
    float max;

    // These are the indices of vertices that
    // contributed to minimum or maximum projection value.
    // In context of collision detection,
    // this is the the index of the vertex most relavant in collision,
    // such as a penetration point.
    int min_vertex_index;
    int max_vertex_index;

    bool IsSeparated(const ProjectionRange& other) const
    {
        return min > other.max || max < other.min;
    }
};

struct Penetration
{
    LineSegment edge;
    float depth;
    int involved_vertex_index;

    /**
     * @brief Comparison by penetration depth.
     * @note std::min requires this operator.
     */
    bool operator<(const Penetration& other) const
    {
        return depth < other.depth;
    }
};

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
    virtual bool IsPointInside(const Vec3& local_point) const override;
    virtual float Area() const override;
    virtual Vec3 CenterOfMass() const override;

    virtual sf::Shape& SFMLShape() override;
    virtual const sf::Shape& SFMLShape() const override;

    virtual std::optional<CollisionInfo> CheckCollision(const ICollider* other) const override;
    virtual std::optional<CollisionInfo> CheckCollisionAccept(const Circle* other) const override;
    virtual std::optional<CollisionInfo> CheckCollisionAccept(const ConvexPolygon* other) const override;

    const std::vector<Vec3>& Vertices() const;
    const std::vector<LineSegment>& Edges() const;

    /**
     * @brief Find the vertices that give maximum or minumum projection
     *        onto the given direction vector.
     * 
     * @see ConvexPolygon::FindMinimumPenetration()
     */
    ProjectionRange Projection(const Vec3& local_direction) const;

    /**
     * @brief Find the edge which gives smallest penetration depth
     *        of the @p other polygon along edge's normal vector.
     * 
     * @note If two polygons are separable with an axis
     *       parallel to an edge, an empty optional is returned.
     * 
     * @see ConvexPolygon::CheckCollisionAccept(const ConvexPolygon* other)
     */
    std::optional<Penetration> FindMinimumPenetration(const ConvexPolygon* other) const;

    /**
     * @brief Between the two edges that contain vertex at index @p involed_vertex_index,
     *        which is more parallel to the given direction vector @p global_dir.
     * 
     * @note This is used to find the incident edge of collision between two polygons.
     * 
     * @see ConvexPolygon::CheckCollisionAccept(const ConvexPolygon* other)
     */
    LineSegment FindMostParallelCollisionEdge(const Vec3& global_dir, int involved_vertex_index) const;
    
private:
    /**
     * @brief Throw exception if the order of vertices is not counter-clockwise.
     */
    void ValidateCounterClockwiseOrder() const;

    std::vector<Vec3> m_vertices;
    std::vector<LineSegment> m_edges;

    // SFML representation.
    sf::ConvexShape m_shape;

    float m_boundary_radius = 0.0f;
    Vec3 m_center_of_mass = {};
};

} // namespace physics

#endif // PHYSICS_CONVEX_POLYGON_H
