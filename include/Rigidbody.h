#ifndef PHYSICS_RIGIDBODY_H
#define PHYSICS_RIGIDBODY_H

#include "ICollider.h"
#include "Vec3.h"
#include "LineSegment.h"
#include "Angle.h"
#include <memory>
#include <optional>

namespace physics
{

/**
 * @brief "DoF" Stands for "Degree of Freedom".
 *        It represents velocity and acceleration of a rigidbody.
 */
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
 
/**
 * @brief A set of physical constants that determine dynamics.
 * 
 * @note These coefficients are supposed to be defined between a pair of objects,
 *       but I just decided to give a value for each object
 *       and use the average value of a colliding pair as an approximation.
 */
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

// Forward declaration for CollisionPair definition.
class Rigidbody;

/**
 * @brief CollisionPair is a wrapper data of CollisionInfo,
 *        which provides pointers to the colliding objects.
 * 
 * @note CollisionInfo does not contain information about 'who'
 */
struct CollisionPair
{
    // Objects who collided with each other.
    Rigidbody* object1;
    Rigidbody* object2;

    // Contact points, normal, and penetration depth.
    CollisionInfo info;
};

/**
 * @brief Rigidbody represents a nondeformable object
 *        which can rotate and translate.
 * 
 * @note Collision detection is handled by colliders.
 * 
 * @note Collision resolution is handled by World class,
 *       using information from rigidbodies.
 * 
 * @note Rigidbody uses Transform of its collider.
 */
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

    physics::Transform& Transform();
    const physics::Transform& Transform() const;

    MaterialProperties& Material();
    const MaterialProperties& Material() const;

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
     * @brief Test if this object has infinite mass and inertia.
     */
    bool IsStatic() const;

    /**
     * @return The velocity of a point inside this rigidbody
     *         expressed in global coordinate system.
    */
    Vec3 GlobalVelocity(const Vec3& local_pos) const;

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
     *       using object pointers stored in CollisionPair,
     *       and applying impulse involves state change in displacement and velocity.
     */
    std::optional<CollisionPair> CheckCollision(Rigidbody& other);

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
     * @brief Reduce the linear and angular velocity by given factor.
     * 
     * @note Damping helps stabilizing simulation with lots of external forces.
     */
    void ApplyDamping(float linear_damping, float angular_damping);

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

    DoF m_velocity;
    DoF m_acceleration;

    /**
     * Reason for storing inverse of mass and inertia:
     * 1. Division by mass or inertia is more frequent than the value itself.
     * 2. Easy to handle infinite mass and inertia,
     *    which is used to represent a static object.
     */
    float m_inv_mass;
    float m_inv_inertia;

};

} // namespace physics

#endif // PHYSICS_RIGIDBODY_H
