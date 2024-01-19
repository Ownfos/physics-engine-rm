#ifndef PHYSICS_UI_H
#define PHYSICS_UI_H

#include "ObjectDragger.h"

namespace physics
{

/**
 * @brief Handles GUI layout and state variables used to record interaction.
 */
class UI
{
public:
    void UpdateGUI();
    void UpdateObjectDragger(ObjectDragger& object_dragger, World& world);

    bool IsUpdateRequired() const;
    bool IsGravityEnabled() const;
    bool IsCollisionEnabled() const;

    float TimeScale() const;
    float DragStrength() const;
    float GravityStrength() const;

    Vec3 MousePosition() const;

private:
    float m_time_scale = 1.0f;
    float m_drag_strength = 0.1f;
    float m_gravity_strength = 9.8f;
    bool m_enable_gravity = true;
    bool m_enable_collision = true;
    bool m_enable_update = true;
    bool m_update_one_step = false;
};

} // namespace physics


#endif // PHYSICS_UI_H
