#ifndef PHYSICS_CONVEX_POLYGON_H
#define PHYSICS_CONVEX_POLYGON_H

#include "ICollider.h"
#include "Vec3.h"
#include "SFML/Graphics/ConvexShape.hpp"
#include <vector>

namespace physics
{

class ConvexPolygon : public ICollider
{
public:
    ConvexPolygon(const std::vector<Vec3>& vertices);

    virtual float BoundaryRadius() const override;
    virtual ColliderType Type() const override;
    virtual sf::Shape& SFMLShape() override;
    virtual const sf::Shape& SFMLShape() const override;
    
private:
    std::vector<Vec3> m_vertices;
    float m_boundary_radius;
    sf::ConvexShape m_shape;
};

} // namespace physics

#endif // PHYSICS_CONVEX_POLYGON_H
