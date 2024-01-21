#include "SpringConnector.h"
#include <cassert>

namespace physics
{

SpringConnector::SpringConnector(std::shared_ptr<World> world)
    : m_world(world)
{}

std::string SpringConnector::Description() const
{
    return "Create or remove springs";
}
std::string SpringConnector::Tooltip() const
{
    return
        "Click an object and release the mouse button\n"
        "on the other end. If start and end points are\n"
        "placed on the same object, all springs connected to\n"
        "that object will be deleted.";
}

void SpringConnector::OnMouseClick(const Vec3& mouse_pos)
{
    m_spring_start = TryPickAnchorPoint(mouse_pos);
}

void SpringConnector::OnMouseDown(const Vec3& mouse_pos)
{
    // noop.
}

void SpringConnector::OnMouseRelease(const Vec3& mouse_pos)
{
    if (m_spring_start.has_value())
    {
        auto spring_end = TryPickAnchorPoint(mouse_pos);

        if (spring_end.has_value())
        {
            // Both end points are on the same object.
            if (m_spring_start->object.get() == spring_end->object.get())
            {
                m_world->RemoveSpringOnObject(m_spring_start->object);
            }
            else
            {
                const auto neutral_distance = (m_spring_start->GlobalPosition() - spring_end->GlobalPosition()).Magnitude();
                m_world->AddSpring(Spring{
                    .start = m_spring_start.value(),
                    .end = spring_end.value(),
                    .neutral_distance = neutral_distance,
                    .coefficient = 10000.0f
                });
            }
        }

        m_spring_start.reset();
    }
}

std::optional<AnchorPoint> SpringConnector::TryPickAnchorPoint(const Vec3& mouse_pos) const
{
    if (auto object = m_world->PickObject(mouse_pos))
    {
        return AnchorPoint{
            .object = object,
            .local_pos = object->Transform().LocalPosition(mouse_pos)
        };
    }

    return {};
}

} // namespace physics
