#ifndef PHYSICS_SPRING_CONNECTOR_H
#define PHYSICS_SPRING_CONNECTOR_H

#include "IMouseAction.h"
#include "World.h"

namespace physics
{

/**
 * @brief Handles connecting two objects with spring
 *        and deleting all springs on an object.
 */
class SpringConnector : public IMouseAction
{
public:
    SpringConnector(std::shared_ptr<World> world);

    virtual std::string Description() const override;
    virtual std::string Tooltip() const override;

    virtual void OnMouseClick(const Vec3& mouse_pos) override;
    virtual void OnMouseDown(const Vec3& mouse_pos) override;
    virtual void OnMouseRelease(const Vec3& mouse_pos) override;

    /**
     * @brief Change the strength of newly created springs.
     * 
     * @param coefficient A positive value representing the strength.
     * 
     * @note This does not affect existing springs!
     */
    void ConfigureSprintCoefficient(float coefficient);

private:
    std::optional<AnchorPoint> TryPickAnchorPoint(const Vec3& mouse_pos) const;

    std::shared_ptr<World> m_world;
    std::optional<AnchorPoint> m_spring_start;

    float m_spring_coefficient = 10000.0f;
};

} // namespace physics



#endif // PHYSICS_SPRING_CONNECTOR_H
