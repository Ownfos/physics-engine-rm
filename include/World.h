#ifndef PHYSICS_WORLD_H
#define PHYSICS_WORLD_H

#include "Rigidbody.h"
#include "Spring.h"

namespace physics
{

/**
 * @brief World is a helper class for managing a group of simulated rigidbodies.
*/
class World
{
public:
    /**
     * @return The list of all rigidbodies managed by this instance.
     * @note The non-const version was required for configuring
     *       SFML shape of the rigidbody's collider,
     *       such as fill color and outline thickness.
     */
    std::vector<std::shared_ptr<Rigidbody>>& Objects();
    const std::vector<std::shared_ptr<Rigidbody>>& Objects() const;

    /**
     * @return The list of all springs managed by this instance.
     */
    const std::vector<Spring>& Springs() const;

    /**
     * @return The list of collisions occured during this time step.
     * @note World::CheckCollision() must be called beforehand!
     */
    const std::vector<CollisionPair>& Collisions() const;

    /**
     * @brief Change the behavior of position adjustment
     *        after a collision is resolved.
     * 
     * @param penetration_allowance The lower bound of penetration depth
     *                              where positional correction starts working.
     * @param correction_ratio The amount of penetration depth we want to adjust.
     *                         0: no correction.
     *                         1: perfect correction.
     * 
     * @note @p correction_ratio must have value in range [0.0, 1.0].
     * @note 'positional correction' refers to the postprocessing step
     *       of collision resolution where both objects are manually
     *       moved away from each other along the collision normal,
     *       so that they no longer overlap after this time step.
     */
    void ConfigurePositionalCorrection(float penetration_allowance, float correction_ratio);

    /**
     * @brief Change the magnitude of linear and angular velocity damping.
     * 
     * @param linear_damping Percentage of current linear velocity.
     * @param angular_damping Percentage of current angular velocity.
     * 
     * @note For example, @p linear_damping of 0.1 will reduce velocity up to 90%.
     */
    void ConfigureDamping(float linear_damping, float angular_damping);

    /**
     * @brief Add and remove a rigidbody from this simulator.
     */
    void AddObject(std::shared_ptr<Rigidbody> object);
    void RemoveObject(const std::shared_ptr<Rigidbody>& object);

    /**
     * @brief Add and remove a spring from this simulator.
     */
    void AddSpring(const Spring& spring);
    void RemoveSpringOnObject(const std::shared_ptr<Rigidbody>& object);

    /**
     * @return The first rigidbody which contains the specified point.
     * 
     * @note If there are multiple candidates, the 'oldest' object is selected.
     */
    std::shared_ptr<Rigidbody> PickObject(const Vec3& pos);

    /**
     * @brief Detect every collision occurrance within this time step.
     * 
     * @note The result can be retreived by calling World::Collisions().
     */
    void CheckCollisions();

    /**
     * @brief Calculate and apply impulse for each collision
     *        so that they can be resolved on the next frame.
     * 
     * @param delta_time The time step of the next Update() invocation.
     * 
     * @note CheckCollisions() must be preceded.
     */
    void ResolveCollisions(float delta_time);

    /**
     * @brief Update position and velocity of all objects.
     * 
     * @param delta_time The time step used in explicit euler integration.
     */
    void Update(float delta_time);

private:
    /**
     * @brief List of all registered rigidbodies.
     */
    std::vector<std::shared_ptr<Rigidbody>> m_objects;

    /**
     * @brief List of all registered springs.
     */
    std::vector<Spring> m_springs;

    /**
     * @brief Stores all collisions detected during this time step.
     *        This gets overwritten whenever World::CheckCollision() is called.
     */
    std::vector<CollisionPair> m_collisions;

    /**
     * @brief Parameters for positional correction.
     * @see World::ConfigurePositionalCorrection()
     */
    float m_penetration_allowance = 0.05f;
    float m_correction_ratio = 0.4f;

    /**
     * @brief Parameters for velocity damping.
     * @see World::ConfigureDamping()
     */
    float m_linear_damping = 0.0f;
    float m_angular_damping = 0.0f;
};

} // namespace physics


#endif // PHYSICS_WORLD_H
