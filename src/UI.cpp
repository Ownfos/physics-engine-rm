#include "UI.h"
#include "imgui.h"

namespace physics
{

void UI::UpdateGUI()
{
    ImGui::Begin("Options");
    ImGui::SliderFloat("time scale", &m_time_scale, 0.01f, 1.0f);
    ImGui::Checkbox("enable gravity", &m_enable_gravity);
    if (m_enable_gravity)
    {
        ImGui::SliderFloat("gravity strength", &m_gravity_strength, 0.0f, 10.0f);
    }
    ImGui::SliderFloat("drag strength", &m_drag_strength, 0.1, 0.5);
    ImGui::Checkbox("resolve collision", &m_enable_collision);
    ImGui::Checkbox("auto update", &m_enable_update);
    m_update_one_step = m_enable_update ? false : ImGui::Button("manual update");
    ImGui::End();
}

void UI::UpdateObjectDragger(ObjectDragger& object_dragger, World& world)
{
    const auto mouse_pos = MousePosition();
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left))
    {
        object_dragger.OnMouseClick(mouse_pos, world);
    }
    else if (ImGui::IsMouseDown(ImGuiMouseButton_Left))
    {
        object_dragger.OnMouseDown(mouse_pos);
    }
    else if (ImGui::IsMouseReleased(ImGuiMouseButton_Left))
    {
        object_dragger.OnMouseRelease();
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

Vec3 UI::MousePosition() const
{
    auto pos = ImGui::GetMousePos();
    return {pos.x, pos.y};
}

} // namespace physics
