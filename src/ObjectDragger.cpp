#include "ObjectDragger.h"
#include <cassert>

namespace physics
{

ObjectDragger::ObjectDragger(std::shared_ptr<World> world)
    : m_world(world)
{}

std::string ObjectDragger::Description() const
{
    return "Drag objects";
}
std::string ObjectDragger::Tooltip() const
{
    return "Click and drag an object to pull it";
}

void ObjectDragger::OnMouseClick(const Vec3& mouse_pos)
{
    if (m_picked_object = m_world->PickObject(mouse_pos))
    {
        // Do not select static objects.
        if (m_picked_object->IsStatic())
        {
            m_picked_object.reset();
        }
        else
        {
            // Record the local coordianate of the point we just clicked.
            m_picked_offset = m_picked_object->Transform().LocalPosition(mouse_pos);
        }
    }
}

void ObjectDragger::OnMouseDown(const Vec3& mouse_pos)
{
    if (m_picked_object)
    {
        m_drag_vector = mouse_pos - m_picked_object->Collider()->Transform().GlobalPosition(m_picked_offset);
    }
}

void ObjectDragger::OnMouseRelease(const Vec3& mouse_pos)
{
    m_picked_object.reset();
}

void ObjectDragger::ApplyDraggingForce(float drag_strength, float time_step)
{
    assert(IsObjectSelected());

    const auto impact_point = PickedPoint() - m_picked_object->Transform().Position();
    const auto force = DragVector() * drag_strength / m_picked_object->InverseMass();
    m_picked_object->ApplyImpulse(impact_point, force, time_step);
}

bool ObjectDragger::IsObjectSelected() const
{
    return m_picked_object != nullptr;
}

Vec3 ObjectDragger::PickedPoint() const
{
    assert(IsObjectSelected());

    return m_picked_object->Transform().GlobalPosition(m_picked_offset);
}

Vec3 ObjectDragger::DragVector() const
{
    assert(IsObjectSelected());
    
    return m_drag_vector;
}

} // namespace physics
