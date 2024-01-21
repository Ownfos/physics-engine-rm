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

    ImGui::SeparatorText("Mouse Right Click");
    for (int i = 0; i < m_mouse_actions.size(); ++i)
    {
        ImGui::RadioButton(m_mouse_actions[i]->Description().c_str(), &m_active_mouse_action_index, i);
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip(m_mouse_actions[i]->Tooltip().c_str());
        }
    }
    ImGui::NewLine();

    ImGui::SeparatorText("Basics");
    ImGui::SliderFloat("time scale", &m_time_scale, 0.01f, 1.0f);
    ImGui::SliderFloat("dragging strength", &m_drag_strength, 0.1f, 0.5f);
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

    ImGui::SeparatorText("Spring");
    ImGui::SliderFloat("spring coefficient", &m_spring_coefficient, 1000.0f, 50000.0f);
    ImGui::NewLine();

    
    ImGui::SeparatorText("Velocity Damping");
    ImGui::SliderFloat("linear damping", &m_linear_damping, 0.0f, 0.1f);
    ImGui::SliderFloat("angular damping", &m_angular_damping, 0.0f, 0.1f);
    ImGui::NewLine();

    ImGui::End();
}

void UI::HandleMouseAction()
{
    // Do nothing if no action is registered yet.
    if (m_mouse_actions.empty())
    {
        return;
    }

    const auto mouse_pos = MousePosition();
    auto& action = m_mouse_actions[m_active_mouse_action_index];
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Right))
    {
        action->OnMouseClick(mouse_pos);
    }
    else if (ImGui::IsMouseDown(ImGuiMouseButton_Right))
    {
        action->OnMouseDown(mouse_pos);
    }
    else if (ImGui::IsMouseReleased(ImGuiMouseButton_Right))
    {
        action->OnMouseRelease(mouse_pos);
    }
}

void UI::AddMouseActionType(std::shared_ptr<IMouseAction> mouse_action)
{
    m_mouse_actions.push_back(mouse_action);
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

float UI::SpringCoefficient() const
{
    return m_spring_coefficient;
}

Vec3 UI::MousePosition() const
{
    auto pos = ImGui::GetMousePos();
    return {pos.x, pos.y};
}

} // namespace physics
