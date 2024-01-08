#ifndef PHYSICS_GIZMO_H
#define PHYSICS_GIZMO_H

#include "SFML/Graphics/RectangleShape.hpp"
#include "Vec3.h"

namespace physics
{

class Gizmo
{
public:
    Gizmo();

    sf::Shape& Point(const Vec3& pos, const sf::Color& color = sf::Color::Black);
    sf::Shape& Direction(const Vec3& pos, const Vec3& dir, const sf::Color& color = sf::Color::Black);

private:
    sf::RectangleShape m_dot;
    sf::RectangleShape m_line;
};

} // namespace physics

#endif // PHYSICS_GIZMO_H
