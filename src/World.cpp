#include "World.h"
#include <cassert>

namespace physics
{

std::vector<std::shared_ptr<Rigidbody>>& World::Objects()
{
    return m_objects;
}

const std::vector<std::shared_ptr<Rigidbody>>& World::Objects() const
{
    return m_objects;
}

const std::vector<CollisionPair>& World::Collisions() const
{
    return m_collisions;
}

void World::ConfigurePositionalCorrection(float penetration_allowance, float correction_ratio)
{
    assert(penetration_allowance >= 0.0f);
    assert(correction_ratio >= 0.0f && correction_ratio <= 1.0f);

    m_penetration_allowance = penetration_allowance;
    m_correction_ratio = correction_ratio;
}

void World::AddObject(std::shared_ptr<Rigidbody> object)
{
    m_objects.push_back(object);
}

void World::RemoveObject(const std::shared_ptr<Rigidbody>& object)
{
    m_objects.erase(std::find(m_objects.begin(), m_objects.end(), object));
}

std::shared_ptr<Rigidbody> World::PickObject(const Vec3& pos)
{
    for (const auto& obj : m_objects)
    {
        if (obj->IsPointInside(pos))
        {
            return obj;
        }
    }
    return {};
}

void World::CheckCollisions()
{
    // Clear previous collision records.
    m_collisions.clear();

    // Iterate over all possible pairs.
    const auto num_obj = m_objects.size();
    for (int i = 0; i < num_obj; ++i)
    {
        for (int j = i + 1; j < num_obj; ++j)
        {
            // Record every collision occurrance.
            if (auto collision = m_objects[i]->CheckCollision(*m_objects[j]))
            {
                m_collisions.push_back(collision.value());
            }
        }
    }
}

void World::ResolveCollisions(float delta_time)
{
    for (const auto& collision : m_collisions)
    {
        // Shorthand notation for objects in contact.
        auto& obj1 = collision.object1;
        auto& obj2 = collision.object2;

        // Choose the physical constants like friction coefficient.
        const auto& mat1 = obj1->Material();
        const auto& mat2 = obj2->Material();
        const auto coef = mat1.Average(mat2);

        // Get the inverse mass.
        const auto inv_mass1 = obj1->InverseMass();
        const auto inv_mass2 = obj2->InverseMass();

        for (const auto& contact : collision.info.contacts)
        {
            // Local coordinates of the position where
            // the collision impulse will be applied to.
            const auto rel_impact_pos1 = contact - obj1->Transform().Position();
            const auto rel_impact_pos2 = contact - obj2->Transform().Position();

            // The global velocity of impact points.
            const auto impact_vel1 = obj1->GlobalVelocity(rel_impact_pos1);
            const auto impact_vel2 = obj2->GlobalVelocity(rel_impact_pos2);

            // Relative impact velocity of object2 in object1's perspective.
            const auto rel_impact_vel = impact_vel2 - impact_vel1;

            // Relative impact velocity along collision normal.
            auto collision_normal = collision.info.normal;
            const auto contact_normal_vel = rel_impact_vel.Dot(collision_normal);

            // Leave the objects if they already moving away.
            // This prevents situation where we get locked inside a wall.
            //
            // ex)     wall <- A  <- B
            //  A collides with the wall and gains velocity towards right.
            //  But B comes in and pushes A back inside to the wall.
            //  Now the velocity of A is headed towards the right side
            //  and our impulse will have opposite effect: pushing A to the wall!
            if (contact_normal_vel > 0.0f)
            {
                continue;
            }

            // TODO: add description on  how this equation was derived...
            const auto denominator =
                inv_mass1 + inv_mass2
                + rel_impact_pos1.Cross(collision_normal).SquaredMagnitude() * obj1->InverseInertia()
                + rel_impact_pos2.Cross(collision_normal).SquaredMagnitude() * obj2->InverseInertia();
            const auto j = -(1 + coef.restitution) * contact_normal_vel / denominator;

            // Reason for dividing impulse for this contact point by contact size:
            //   We might have multiple impact points per collision!
            //   To approximate total energy conservation,
            //   the average impulse of all local impulse per impact point must be used.
            const auto impulse = collision_normal * j / collision.info.contacts.size();

            // Due to the law of action and reaction,
            // the magnitude of impulse is same but the direction is opposite.
            obj1->ApplyImpulse(rel_impact_pos1, -impulse, delta_time);
            obj2->ApplyImpulse(rel_impact_pos2, impulse, delta_time);
        }

        // Perform positional correction.
        if (collision.info.penetration_depth > m_penetration_allowance)
        {
            const auto required_translation =
                // The direction where we need separation.
                collision.info.normal
                // Allow some penetration for simulation stability.
                * (collision.info.penetration_depth - m_penetration_allowance)
                // Smoothly resolve overlapping issue.
                // Again, for simulation stability.
                * m_correction_ratio;
            
            // The total translation required to separate objects
            // is distributed according to the ratio of inverse mass.
            // This makes heavy objects stable, while light objects move more.
            const auto inv_mass_ratio = inv_mass1 / (inv_mass1 + inv_mass2);
            obj1->Transform().AddPosition(- required_translation * inv_mass_ratio);
            obj2->Transform().AddPosition(required_translation * (1.0f - inv_mass_ratio));
        }
    }
}

void World::Update(float delta_time)
{
    for (const auto& obj : m_objects)
    {
        obj->Update(delta_time);
    }
}


} // namespace physics
