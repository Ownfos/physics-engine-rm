#ifndef PHYSICS_CIRCLE_H
#define PHYSICS_CIRCLE_H

#include "ICollider.h"
#include "SFML/Graphics/CircleShape.hpp"

namespace physics
{

class Circle : public ICollider
{
public:
    Circle(float radius);

    virtual float BoundaryRadius() const override;
    virtual ColliderType Type() const override;
    virtual sf::Shape& SFMLShape() override;
    virtual const sf::Shape& SFMLShape() const override;
    
private:
    sf::CircleShape m_shape;
};

} // namespace physics

#endif // PHYSICS_CIRCLE_H
