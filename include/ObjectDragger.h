#ifndef PHYSICS_OBJECT_DRAGGER_H
#define PHYSICS_OBJECT_DRAGGER_H

#include "World.h"

namespace physics
{

/**
 * @brief Handles picking an object in the scene with mouse
 *        and applying force towards the cursor.
 */
class ObjectDragger
{
public:
    void OnMouseClick(const Vec3& mouse_pos, World& world);
    void OnMouseDown(const Vec3& mouse_pos);
    void OnMouseRelease();

    void ApplyDraggingForce(float drag_strength, float time_step);

    bool IsObjectSelected() const;

    Vec3 PickedPoint() const;
    Vec3 DragVector() const;

private:
    std::shared_ptr<Rigidbody> m_picked_object;
    Vec3 m_picked_offset;
    Vec3 m_drag_vector;
};

} // namespace physics



#endif // PHYSICS_OBJECT_DRAGGER_H
