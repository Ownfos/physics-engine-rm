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

private:
    std::optional<AnchorPoint> TryPickAnchorPoint(const Vec3& mouse_pos) const;

    std::shared_ptr<World> m_world;
    std::optional<AnchorPoint> m_spring_start;
};

} // namespace physics



#endif // PHYSICS_SPRING_CONNECTOR_H
