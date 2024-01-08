#ifndef PHYSICS_WORLD_H
#define PHYSICS_WORLD_H

#include "Rigidbody.h"

namespace physics
{

class World
{
public:
    std::vector<std::shared_ptr<Rigidbody>>& Objects();
    const std::vector<std::shared_ptr<Rigidbody>>& Objects() const;
    const std::vector<CollisionInfo>& Collisions() const;

    void AddObject(std::shared_ptr<Rigidbody> object);
    void RemoveObject(const std::shared_ptr<Rigidbody>& object);
    std::shared_ptr<Rigidbody> PickObject(const Vec3& pos);

    void CheckCollisions();
    void ResolveCollisions(float delta_time);
    void Update(float delta_time);

private:
    std::vector<std::shared_ptr<Rigidbody>> m_objects;
    std::vector<CollisionInfo> m_collisions;
};

} // namespace physics


#endif // PHYSICS_WORLD_H
