#ifndef PHYSICS_RIGIDBODY_H
#define PHYSICS_RIGIDBODY_H

#include "ICollider.h"
#include "CollisionInfo.h"
#include "Vec3.h"
#include "LineSegment.h"
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

// A set of physical constants that determine dynamics.
// 
// @note These coefficients are supposed to be defined between a pair of objects,
//       but I just decided to give a value for each object
//       and use the average as an approximation.
struct MaterialProperties
{
    /**
     * @brief Decides how 'bouncy' an object is.
     * @note 0: perfectly inelastic collision
     * @note 1: perfectly elastic collision
     */
    float restitution;

    /**
     * @brief Decides the lower bound of horizontal force
     *        required to make a static object move.
     * @note While Ft < Fn * @p static_friction,
     *       -Ft is applied to cancel external tangent force Ft.
     * @note Ft: force along tangent direction.
     * @note Fn: force along normal direction.
     */
    float static_friction;

    /**
     * @brief Decides the ratio of vertical force translated into friction force.
     * @note This coefficient is used whenever a contact point
     *       has nonzero relative velocity w.r.t. the other object.
     */
    float dynamic_friction;

    inline MaterialProperties Average(const MaterialProperties& other) const
    {
        return {
            (restitution + other.restitution) / 2.0f,
            (static_friction + other.static_friction) / 2.0f,
            (dynamic_friction + other.dynamic_friction) / 2.0f,
        };
    }
};

class Rigidbody
{
public:
    Rigidbody(
        std::shared_ptr<ICollider> collider,
        const MaterialProperties& material,
        float mass,
        float inertia
    );

    ICollider* Collider();
    const ICollider* Collider() const;
    const MaterialProperties& Material() const;
    const Vec3& Position() const;
    const Vec3& Rotation() const;
    const Vec3& LinearVelocity() const;
    const Vec3& AngularVelocity() const;

    float InverseMass() const;
    float InverseInertia() const;

    /**
     * @param global_pos The point we want to test, which is
     *              expressed in global coordinate system.
     * @return True if the point resides within the collider,
     *         considering object's transform.
     */
    bool IsPointInside(const Vec3& global_pos) const;

    /**
     * @brief Helper functions for transforming coordinates
     *        between global and local coordinate systems.
     */
    Vec3 LocalToGlobal(const Vec3& local_pos) const;
    Vec3 GlobalToLocal(const Vec3& global_pos) const;
    LineSegment LocalToGlobal(const LineSegment& local_edge) const;
    LineSegment GlobalToLocal(const LineSegment& global_edge) const;

    /**
     * @return The velocity of a point inside this rigidbody
     *         expressed in global coordinate system.
    */
    Vec3 GlobalVelocity(const Vec3& local_pos) const;

    /**
     * @note @p new_position should be on a 2D plane (i.e., new_position.z == 0)
     */
    void SetPosition(const Vec3& new_position);
    void MovePosition(const Vec3& offset);

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
     * @param rel_impact_pos Point of impact w.r.t. this object's origin.
     * @param impulse Desired net change in momentum.
     * @param delta_time The time step of the following update step.
     * 
     * @warning The direction of @p impact and @p rel_impact_pos should be global!
     *          Suppose we have a circle with radius = 1 and rotation = 90 degrees counter-clockwise.
     *          If we want to push local point (1, 0) of that circle to the right,
     *          @p rel_impact_pos should be (0, 1) and the direction of @p impact should be (1, 0),
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
    MaterialProperties m_material;

    DoF m_displacement;
    DoF m_velocity;
    DoF m_acceleration;

    float m_inv_mass;
    float m_inv_inertia;

};

} // namespace physics

#endif // PHYSICS_RIGIDBODY_H
