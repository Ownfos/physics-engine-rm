#include "Circle.h"
#include "ConvexPolygon.h"
#include "Angle.h"

namespace physics
{

Circle::Circle(float radius)
    : m_shape(radius)
{
    // sf::CircleShape has origin on the corner,
    // so we should adjust it to the origin.
    m_shape.setOrigin({radius, radius});
}

float Circle::BoundaryRadius() const
{
    return m_shape.getRadius();
}

ColliderType Circle::Type() const
{
    return ColliderType::Circle;
}

bool Circle::IsPointInside(const Vec3& local_point) const
{
    return local_point.Magnitude() <= BoundaryRadius();
}

float Circle::Area() const
{
    return BoundaryRadius() * BoundaryRadius() * pi;
}

Vec3 Circle::CenterOfMass() const
{
    return Transform().Position();
}

sf::Shape& Circle::SFMLShape()
{
    return m_shape;
}

const sf::Shape& Circle::SFMLShape() const
{
    return m_shape;
}

std::optional<CollisionInfo> Circle::CheckCollision(const ICollider* other) const
{
    return other->CheckCollisionAccept(this);
}

std::optional<CollisionInfo> Circle::CheckCollisionAccept(const Circle* other) const
{

    // My position  w.r.t. other object's perspective.
    const auto other_to_this = Transform().Position() - other->Transform().Position();
    
    // The minimum distance required to separate two circles.
    // If they are just one step away from collision,
    // this would be the distance between their corresponding center.
    const auto min_separation_distance = BoundaryRadius() + other->BoundaryRadius();

    // Calculate the normal vector.
    auto result = CollisionInfo{};
    if (other_to_this.IsZero())
    {
        // Since two circles are on the exactly same position,
        // the direction of the impulse doesn't matter.
        // 
        // Just arbitrarily choose (1, 0) as the normal vector
        // so that they push each other horizontally.
        result.normal = {1.0f, 0.0f};
    }
    else
    {
        // 'this' must move away from 'other'.
        result.normal = other_to_this;
        result.normal.Normalize();
    }

    // The distance required to separate two circles.
    result.penetration_depth = min_separation_distance - other_to_this.Magnitude();

    // Case 1) collision happened!
    if (result.penetration_depth > 0.0f)
    {
        // The intersection between the 'other' circle
        // and the line connecting centers of 'this' and 'other'.
        const auto contact_point = other->Transform().Position() + result.normal * other->BoundaryRadius();
        result.contacts.push_back(contact_point);
        return result;
    }
    // Case 2) they were too far from each other...
    else
    {
        return {};
    }
}

std::optional<CollisionInfo> Circle::CheckCollisionAccept(const ConvexPolygon* other) const
{
    // Key idea: there are two cases where collision occurs.
    // 1. circle's center is inside the polygon.
    // 2. circle's center is outside the polygon,
    //    but the distance is shorter than its radius.

    // The position of the circle's center w.r.t. the polygon.
    const auto circle_rel_pos = other->Transform().LocalPosition(Transform().Position());

    // Case 1) check if the center of the circle is within the polygon.
    const auto is_circle_inside_poly = other->IsPointInside(circle_rel_pos);

    // From now on, every calculation will be done under polygon's coordinate system.
    auto result = std::optional<CollisionInfo>{};
    const auto circle_radius = BoundaryRadius();
    for (const auto& edge : other->Edges())
    {
        // Case 2) check if the circle is close enough to the polygon's boundary,
        //         but the circle's center is still outside of the polygon.
        const auto closest_point = edge.FindClosestPointOnLine(circle_rel_pos);
        const auto edge_to_circle_center = circle_rel_pos - closest_point;
        const auto dist_from_edge = edge_to_circle_center.Magnitude();

        const auto is_circle_outside_edge = edge_to_circle_center.Dot(edge.Normal()) > 0.0f;
        const auto is_circle_touching_edge = is_circle_outside_edge && dist_from_edge < circle_radius;


        // If either condition for collision is satisfied,
        // record the minimum penetration depth and the collision normal.
        if (is_circle_inside_poly || is_circle_touching_edge)
        {

            // Now calculate the normal and penetration depth,
            // depending on the collision condition (either case 1 or case 2).
            auto collision = CollisionInfo{};
            if (is_circle_inside_poly)
            {
                // Move the circle out of the polygon along edge normal.
                collision.normal = other->Transform().GlobalDirection(edge.Normal());

                // Choose the circle's center as impact point.
                // Reason for not using circle's boundary point:
                // 1. Impact point becomes noncontinuous on the border of the polygon.
                // 2. The boundary point might be on the outside of the polygon,
                //    in case the circle is way larger than the other.
                collision.contacts.push_back(Transform().Position());

                // However, penetration depth is the minimum translation distance
                // required to separate two objects.
                // Therefore, this must take radius into account.
                collision.penetration_depth = circle_radius + dist_from_edge;
            }
            else
            {
                // When circle collides with the corner, especially on a sharp one,
                // edge normal can greatly differ depending on the selected edge.
                // To prevent such noncontinuous collision normal,
                // we use edge_to_circle_center instead of edge normal.
                collision.normal = other->Transform().GlobalDirection(edge_to_circle_center);
                collision.normal.Normalize();

                // Use the point on the edge, closest to the circle's center, as impact point.
                collision.contacts.push_back(other->Transform().GlobalPosition(closest_point));

                // The circle barely touches the polygon when dist_from_edge == circle_radius
                // and in this case, circle_radius is always greater than dist_from_edge.
                collision.penetration_depth = circle_radius - dist_from_edge;
            }

            // Keep recording the collision information with minimum penetration depth.
            if (!result.has_value() || result->penetration_depth > collision.penetration_depth)
            {
                result = collision;
            }
        }
    }

    return result;
}

} // namespace physics
