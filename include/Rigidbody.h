#ifndef PHYSICS_RIGIDBODY_H
#define PHYSICS_RIGIDBODY_H

#include "ICollider.h"
#include "CollisionInfo.h"
#include "Vec3.h"
#include "Angle.h"
#include <memory>
#include <optional>

namespace physics
{

// "DoF" Stands for "Degree of Freedom".
// It represents displacement and velocity of a rigidbody.
struct DoF
{
    Vec3 linear;

    // Only uses the z-axis.
    // Treating angular disposition & velocity as a vector
    // allows easier calculation of following things:
    // - global linear velocity of a local point inside a rigidbody
    // - torque from a linear impulse on a local point inside a rigidbody
    //
    // @note unit is radian, not degrees.
    Vec3 angular;
};

class Rigidbody
{
public:
    Rigidbody(std::shared_ptr<ICollider> collider, float mass, float inertia);

    const ICollider* Collider() const;
    const Vec3& Position() const;
    const Vec3& Rotation() const;
    const Vec3& LinearVelocity() const;
    const Vec3& AngularVelocity() const;

    /**
     * @note @p new_position should be on a 2D plane (i.e., new_position.z == 0)
     */
    void SetPosition(const Vec3& new_position);

    /**
     * @note rotation vectors are treated as 3D vectors with z-component.
     */
    void SetRotation(Radian new_rotation);

    /**
     * @note @p new_mass should not be negative.
     */
    void SetMass(float new_mass);

    /**
     * @note @p new_inertia should not be negative.
     */
    void SetInertia(float new_inertia);

    /**
     * @brief Make the object immutable to external force.
     *        This is identical to giving infinite mass and inertia.
     */
    void MakeObjectStatic();

    /**
     * @return The SFML representation of this object.
     * 
     * @note Translation and rotation are applied automatically.
     */
    sf::Shape& SFMLShape();

    /**
     * @return True if the distance between these objects
     *         are larger than the sum of their boundary radius,
     *         which means that they are impossible to collide.
     */
    bool IsOutOfBoundaryRadius(const Rigidbody& other) const;

    /**
     * @brief Return the collision information if the two objects collided.
     * 
     * @note This function is non-const because we need to apply impulse
     *       using object pointers stored in CollisionInfo,
     *       and applying impulse involves state change in displacement and velocity.
     */
    std::optional<CollisionInfo> CheckCollision(Rigidbody& other);

    /**
     * @brief Assuming that a constant force will be applied on a local point @p impact_pos,
     *        accumulate linear and angular force corresponding to the @p impulse vector.
     * 
     * @param rel_impact_pos Point of impact w.r.t. this object's coordinate.
     * @param impulse Desired net change in momentum.
     * @param delta_time The time step of the following update step.
     * 
     * @note J = ∫(F * dt) = F * Δt, if F is constant over time.
     */
    void ApplyImpulse(const Vec3& rel_impact_pos, const Vec3& impulse, float delta_time);

    /**
     * @brief Perform explicit euler integration on linear and angular disposition.
     * 
     * @param delta_time The time step between previous and current frame.
     * 
     * @note @p delta_time should be identical to the value used on ApplyImpulse().
     */
    void Update(float delta_time);

private:
    std::shared_ptr<ICollider> m_collider;

    DoF m_displacement;
    DoF m_velocity;
    DoF m_acceleration;

    float m_inv_mass;
    float m_inv_inertia;
};

} // namespace physics

#endif // PHYSICS_RIGIDBODY_H
