#include "PolygonDrawer.h"
#include "ConvexPolygon.h"
#include <cassert>

namespace physics
{

PolygonDrawer::PolygonDrawer(std::shared_ptr<World> world, float draw_finish_distance)
    : m_world(world), m_draw_finish_distance(draw_finish_distance)
{}

std::string PolygonDrawer::Description() const
{
    return "draw new convex polygon object";
}

void PolygonDrawer::OnMouseClick(const Vec3& mouse_pos)
{
    if (IsDrawingFinished(mouse_pos))
    {
        TrySpawnObject();
        ClearVertices();
    }
    else
    {
        AppendVertex(mouse_pos);
    }
}

void PolygonDrawer::OnMouseDown(const Vec3& mouse_pos)
{
    // Noop.
}

void PolygonDrawer::OnMouseRelease(const Vec3& mouse_pos)
{
    // Noop.
}

const std::vector<Vec3>& PolygonDrawer::CurrentVertices() const
{
    return m_vertices;
}

bool PolygonDrawer::IsDrawingFinished(const Vec3& clicked_pos) const
{
    // We need at least three vertices for a polygon.
    if (m_vertices.size() < 3)
    {
        return false;
    }

    // The clicked point should be close enough
    // to the first vertex, in order to finish drawing.
    const auto dist_from_first_vert = (m_vertices.front() - clicked_pos).Magnitude();
    return dist_from_first_vert < m_draw_finish_distance;
}

void PolygonDrawer::TrySpawnObject()
{
    try
    {
        // Since vertices are recorded using global coordinate,
        // we need to adjust them w.r.t. the center of mass.
        // This step can be thought as moving the origin of this object
        // from global origin (0, 0) to the object's center (i.e., 'center of mass').
        const auto center_of_mass = m_vertex_sum / m_vertices.size();
        for (auto& vertex : m_vertices)
        {
            vertex -= center_of_mass;
        }

        // The constructor will throw exception if
        // the vertices were not ordered counter-clockwise.
        auto polygon_shape = std::make_shared<ConvexPolygon>(m_vertices);
        
        auto default_mat = MaterialProperties{
            .restitution = 0.7f,
            .static_friction = 0.6f,
            .dynamic_friction = 0.3f
        };

        // Estimate mass and inertia based on the polygon's surface area.
        const auto area = polygon_shape->Area();
        const auto mass = area;
        const auto inertia = area * area;
        auto object = std::make_shared<Rigidbody>(polygon_shape, default_mat, mass, inertia);
        object->Transform().SetPosition(center_of_mass);

        // Add the newly created object to the simulator.
        m_world->AddObject(object);
    }
    catch(...)
    {
        // TODO: handle exception.
    }
}

void PolygonDrawer::AppendVertex(const Vec3& vertex)
{
    m_vertices.push_back(vertex);
    m_vertex_sum += vertex;
}

void PolygonDrawer::ClearVertices()
{
    m_vertices.clear();
    m_vertex_sum = {};
}

} // namespace physics
