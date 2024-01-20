#ifndef PHYSICS_UI_H
#define PHYSICS_UI_H

#include "IMouseAction.h"
#include <vector>
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
     * @brief Add an option for mouse click handler.
     * 
     * @param mouse_action A pointer to an instance of IMouseAction child class.
     */
    void AddMouseActionType(std::shared_ptr<IMouseAction> mouse_action);

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

    // List of all possible actions for mouse clicks.
    // One of them will be chosen and set as m_active_mouse_action.
    std::vector<std::shared_ptr<IMouseAction>> m_mouse_actions;

    // The action executed in response to mouse clicks.
    int m_active_mouse_action_index = 0;
};

} // namespace physics


#endif // PHYSICS_UI_H
