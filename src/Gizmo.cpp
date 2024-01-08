#include "Gizmo.h"
#include "Angle.h"

namespace physics
{

Gizmo::Gizmo()
    : m_dot({4, 4}), m_line({10, 2})
{
    m_dot.setOrigin({2, 2});
    m_line.setOrigin({0, 1});
}

sf::Shape& Gizmo::Point(const Vec3& pos, const sf::Color& color)
{
    m_dot.setPosition(pos.x, pos.y);
    m_dot.setScale(1, 1);
    m_dot.setFillColor(color);

    return m_dot;
}

sf::Shape& Gizmo::Direction(const Vec3& pos, const Vec3& dir, const sf::Color& color)
{
    m_line.setPosition(pos.x, pos.y);
    m_line.setRotation(rad2deg(std::atan2f(dir.y, dir.x)));
    m_line.setFillColor(color);
    
    return m_line;
}

} // namespace physics
