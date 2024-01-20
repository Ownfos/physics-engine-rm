#ifndef PHYSICS_UI_H
#define PHYSICS_UI_H

#include "IMouseAction.h"
#include <memory>

namespace physics
{

/**
 * @brief Handles GUI layout and state variables used to record interaction.
 */
class UI
{
public:
    /**
     * @brief Render ImGUI elements and handle mouse action if any.
     * 
     * @note This should be called after ImGui::SFML::Update() and before ImGui::SFML::Render().
     */
    void Update();

    /**
     * @brief Change how we react to the mouse clicks.
     * 
     * @param mouse_action A pointer to an instance of IMouseAction child class.
     */
    void SetMouseAction(std::shared_ptr<IMouseAction> mouse_action);

    bool IsUpdateRequired() const;
    bool IsGravityEnabled() const;
    bool IsCollisionEnabled() const;

    float TimeScale() const;
    float DragStrength() const;
    float GravityStrength() const;
    float LinearDamping() const;
    float AngularDamping() const;

    Vec3 MousePosition() const;

private:
    void DrawUI();
    void HandleMouseAction();

    bool m_enable_gravity = true;
    bool m_enable_collision = true;
    bool m_enable_update = true;
    bool m_update_one_step = false;

    float m_time_scale = 1.0f;
    float m_drag_strength = 0.2f;
    float m_gravity_strength = 9.8f;
    float m_linear_damping = 0.0f;
    float m_angular_damping = 0.0f;

    // The action executed in response to mouse clicks.
    std::shared_ptr<IMouseAction> m_mouse_action;
};

} // namespace physics


#endif // PHYSICS_UI_H
