#ifndef PHYSICS_GIZMO_H
#define PHYSICS_GIZMO_H

#include "SFML/Graphics/RectangleShape.hpp"
#include "Vec3.h"

namespace physics
{

/**
 * @brief Gizmo is a helper class for drawing debug-purpose shapes using SFML.
 *        It provides a reference to an SFML shape that represents
 *        a point or a directional vector, using global coordinate.
 */
class Gizmo
{
public:
    Gizmo();

    sf::Shape& Point(const Vec3& pos, const sf::Color& color = sf::Color::Black);
    sf::Shape& Direction(const Vec3& pos, const Vec3& dir, const sf::Color& color = sf::Color::Black);
    sf::Drawable& Line(const Vec3& start, const Vec3& end, const sf::Color& color = sf::Color::Black);

private:
    sf::RectangleShape m_dot;
    sf::RectangleShape m_direction;
    sf::VertexArray m_line;
};

} // namespace physics

#endif // PHYSICS_GIZMO_H
