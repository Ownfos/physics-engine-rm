#include "Gizmo.h"
#include "Angle.h"

namespace physics
{

Gizmo::Gizmo()
    : m_dot({4, 4}), m_direction({10, 2}), m_line(sf::LineStrip, 2)
{
    m_dot.setOrigin({2, 2});
    m_direction.setOrigin({0, 1});
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
    m_direction.setPosition(pos.x, pos.y);
    m_direction.setRotation(rad2deg(std::atan2f(dir.y, dir.x)));
    m_direction.setFillColor(color);
    
    return m_direction;
}

sf::Drawable& Gizmo::Line(const Vec3& start, const Vec3& end, const sf::Color& color)
{
    m_line[0].position = sf::Vector2f(start.x, start.y);
    m_line[0].color = color;
    m_line[1].position = sf::Vector2f(end.x, end.y);
    m_line[1].color = color;

    return m_line;
}

} // namespace physics
