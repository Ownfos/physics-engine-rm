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

const std::vector<Spring>& World::Springs() const
{
    return m_springs;
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

void World::ConfigureDamping(float linear_damping, float angular_damping)
{
    assert(linear_damping >= 0.0f && linear_damping < 1.0f);
    assert(angular_damping >= 0.0f && angular_damping < 1.0f);

    m_linear_damping = linear_damping;
    m_angular_damping = angular_damping;
}

void World::AddObject(std::shared_ptr<Rigidbody> object)
{
    m_objects.push_back(object);
}

void World::RemoveObject(const std::shared_ptr<Rigidbody>& object)
{
    m_objects.erase(std::find(m_objects.begin(), m_objects.end(), object));
}

void World::AddSpring(const Spring& spring)
{
    m_springs.push_back(spring);
}

void World::RemoveSpringOnObject(const std::shared_ptr<Rigidbody>& object)
{
    // Returns true for a spring connected to the specified object.
    const auto pred = [&object](const auto& spring){
        return spring.start.object == object || spring.end.object == object;
    };

    // Note: erase-remove idiom!
    m_springs.erase(
        std::remove_if(m_springs.begin(), m_springs.end(), pred),
        m_springs.end()
    );
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

Vec3 RelativeImpactVelocity(const Rigidbody* object1, const Rigidbody* object2, const Vec3& rel_impact_pos1, const Vec3& rel_impact_pos2)
{
    // The global velocity of impact points.
    const auto impact_vel1 = object1->GlobalVelocity(rel_impact_pos1);
    const auto impact_vel2 = object2->GlobalVelocity(rel_impact_pos2);

    // Relative impact velocity of object2 in object1's perspective.
    return impact_vel2 - impact_vel1;
}

/**
 * @brief Find the magnitude of an impulse vector that will make
 *        the ratio between relative impact velocity before and after collision
 *        become @p restitution, which is the coefficient of restitution
 *        in the Newton's law of restitution.
 * 
 * @param rel_impact_pos1 Displacement of contact point from object1's center, in global coordinate.
 * @param rel_impact_pos2 Displacement of contact point from object2's center, in global coordinate.
 * @param normal The collision direction.
 * @param restitution Coefficient of restitution.
 *                    1.0 results in velocity exchange,
 *                    while 0.0 results in zero relative velocity.
 */
float CalculateCollisionImpulseMagnitude(const Rigidbody* object1, const Rigidbody* object2, const Vec3& rel_impact_pos1, const Vec3& rel_impact_pos2, const Vec3& normal, float restitution)
{
    const auto velocity_along_normal = RelativeImpactVelocity(object1, object2, rel_impact_pos1, rel_impact_pos2).Dot(normal);

    const auto denominator =
        object1->InverseMass() + object2->InverseMass()
        + rel_impact_pos1.Cross(normal).SquaredMagnitude() * object1->InverseInertia()
        + rel_impact_pos2.Cross(normal).SquaredMagnitude() * object2->InverseInertia();

    return -(1 + restitution) * velocity_along_normal / denominator;
}

void World::ResolveCollisions(float delta_time)
{
    for (const auto& collision : m_collisions)
    {
        // Shorthand notation for objects in contact.
        auto& object1 = collision.object1;
        auto& object2 = collision.object2;

        // Choose the physical constants like friction coefficient.
        const auto coef = object1->Material().Average(object2->Material());

        for (const auto& contact : collision.info.contacts)
        {
            // Local coordinates of the position where
            // the collision impulse will be applied to.
            const auto rel_impact_pos1 = contact - object1->Transform().Position();
            const auto rel_impact_pos2 = contact - object2->Transform().Position();

            const auto normal_impulse_magnitude = CalculateCollisionImpulseMagnitude(object1, object2, rel_impact_pos1, rel_impact_pos2, collision.info.normal, coef.restitution);
            const auto normal_impulse = collision.info.normal * normal_impulse_magnitude;

            // Leave the objects if they already moving away.
            // This prevents situation where we get locked inside a wall.
            //
            // ex)     wall <- A  <- B
            //  A collides with the wall and gains velocity towards right.
            //  But B comes in and pushes A back inside to the wall.
            //  Now the velocity of A is headed towards the right side
            //  and our impulse will have opposite effect: pushing A to the wall!
            if (normal_impulse_magnitude < 0.0f)
            {
                continue;
            }

            // Handling friction!
            // First, find the tangential vector opposite to the relative impact velocity.
            // Note: friction is always resistant (thereby opposite) to the tangential velocity.
            const auto rel_impact_vel = RelativeImpactVelocity(object1, object2, rel_impact_pos1, rel_impact_pos2);
            auto friction_direction = -(rel_impact_vel - rel_impact_vel.Projection(collision.info.normal));
            friction_direction.Normalize();

            // Now that we know the direction of friction force,
            // we need to calculate the magnitude of friction.
            //
            // Instead of resisting to the external forces applied during this time step,
            // we try to correct the nonzero tangential velocity of a collision point, which should have been zero.
            //
            // Reason:
            //   Static friction cannot be handled in a single step.
            //
            //   We first need to find all contacts with zero tangential velocity
            //   and then apply collision impulse, taking static friction into account.
            //
            //   However, our approach assumes that all collisions are independent
            //   and collision resolution is done in arbitrary order.
            //
            //   Therefore, the second best thing we can do is applying additional force
            //   that will make the tangential contact velocity zero.
            //   Except that we have one time step of delay,
            //   it basically does what a static friction would have done.
            //
            // How can we calculate the right amount of force?
            // Well, use the same formula as the regular collision impact!
            // Replacing collision normal to collision tangent, and coefficient of restitution to 0 will work.
            auto tangential_impulse_magnitude = CalculateCollisionImpulseMagnitude(object1, object2, rel_impact_pos1, rel_impact_pos2, friction_direction, 0.0f);

            // If the force required to make tangential contact velocity
            // is greater than the maximum static friction force,
            // we must be using dynamic friction instead.
            const auto max_static_friction_magnitude = normal_impulse_magnitude * coef.static_friction;
            if (tangential_impulse_magnitude > max_static_friction_magnitude)
            {
                tangential_impulse_magnitude = normal_impulse_magnitude * coef.dynamic_friction;
            }
            const auto tangential_impulse = friction_direction * tangential_impulse_magnitude;

            // Reason for dividing impulse for this contact point by contact size:
            //   We might have multiple impact points per collision!
            //   To approximate total energy conservation,
            //   the average impulse of all local impulse per impact point must be used.
            //
            // - example scenario -
            // Suppose two parallel squares are colliding horizontally.
            // If one square is smaller, we will have two contact points on an overlapping edge.
            // This means we apply impulse on two corners!
            // Since each impulse magnitude j is calculated for complete resolution,
            // we need to divide each impulse by 2 so that the sum of them gives the right answer.
            const auto total_impulse = (normal_impulse + tangential_impulse) / collision.info.contacts.size();

            // Due to the law of action and reaction,
            // the magnitude of impulse is same but the direction is opposite.
            object1->ApplyImpulse(rel_impact_pos1, -total_impulse, delta_time);
            object2->ApplyImpulse(rel_impact_pos2, total_impulse, delta_time);
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
            const auto inv_mass_ratio = object1->InverseMass() / (object1->InverseMass() + object2->InverseMass());
            object1->Transform().AddPosition(- required_translation * inv_mass_ratio);
            object2->Transform().AddPosition(required_translation * (1.0f - inv_mass_ratio));
        }
    }
}

void World::Update(float delta_time)
{
    for (auto& spring : m_springs)
    {
        spring.ApplyImpulse(delta_time);
    }

    for (const auto& obj : m_objects)
    {
        obj->Update(delta_time);
        obj->ApplyDamping(m_linear_damping, m_angular_damping);
    }
}


} // namespace physics
