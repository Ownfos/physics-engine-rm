#include "UI.h"
#include "imgui.h"

namespace physics
{

void UI::Update()
{
    DrawUI();
    HandleMouseAction();
}

void UI::DrawUI()
{
    ImGui::Begin("Options");

    ImGui::SeparatorText("Basics");
    ImGui::SliderFloat("time scale", &m_time_scale, 0.01f, 1.0f);
    ImGui::SliderFloat("object dragging strength", &m_drag_strength, 0.1f, 0.5f);
    ImGui::Checkbox("resolve collision", &m_enable_collision);
    ImGui::Checkbox("auto update", &m_enable_update);
    m_update_one_step = m_enable_update ? false : ImGui::Button("manual update");
    ImGui::NewLine();

    ImGui::SeparatorText("Gravity");
    ImGui::Checkbox("enable gravity", &m_enable_gravity);
    if (m_enable_gravity)
    {
        ImGui::SliderFloat("gravity strength", &m_gravity_strength, 0.0f, 10.0f);
    }
    ImGui::NewLine();

    
    ImGui::SeparatorText("Velocity Damping");
    ImGui::SliderFloat("linear damping", &m_linear_damping, 0.0f, 0.1f);
    ImGui::SliderFloat("angular damping", &m_angular_damping, 0.0f, 0.1f);
    ImGui::NewLine();

    ImGui::SeparatorText("Change Click Mode");
    for (auto& mouse_action : m_mouse_actions)
    {
        if (ImGui::Button(mouse_action->Description().c_str()))
        {
            m_active_mouse_action = mouse_action;
        }
    }
    ImGui::NewLine();

    ImGui::End();
}

void UI::HandleMouseAction()
{
    // Do nothing if no action is registered yet.
    if (m_active_mouse_action == nullptr)
    {
        return;
    }

    const auto mouse_pos = MousePosition();
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left))
    {
        m_active_mouse_action->OnMouseClick(mouse_pos);
    }
    else if (ImGui::IsMouseDown(ImGuiMouseButton_Left))
    {
        m_active_mouse_action->OnMouseDown(mouse_pos);
    }
    else if (ImGui::IsMouseReleased(ImGuiMouseButton_Left))
    {
        m_active_mouse_action->OnMouseRelease(mouse_pos);
    }
}

void UI::AddMouseActionType(std::shared_ptr<IMouseAction> mouse_action)
{
    m_mouse_actions.push_back(mouse_action);

    // Use the first action as a default choice.
    if (m_active_mouse_action == nullptr)
    {
        m_active_mouse_action = mouse_action;
    }
}

bool UI::IsGravityEnabled() const
{
    return m_enable_gravity;
}

bool UI::IsUpdateRequired() const
{
    return m_enable_update || m_update_one_step;
}

bool UI::IsCollisionEnabled() const
{
    return m_enable_collision;
}

float UI::TimeScale() const
{
    return m_time_scale;
}

float UI::DragStrength() const
{
    return m_drag_strength;
}

float UI::GravityStrength() const
{
    return m_gravity_strength;
}

float UI::LinearDamping() const
{
    return m_linear_damping;
}

float UI::AngularDamping() const
{
    return m_angular_damping;
}

Vec3 UI::MousePosition() const
{
    auto pos = ImGui::GetMousePos();
    return {pos.x, pos.y};
}

} // namespace physics
