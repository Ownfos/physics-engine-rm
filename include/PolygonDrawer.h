#ifndef PHYSICS_POLYGON_DRAWER_H
#define PHYSICS_POLYGON_DRAWER_H

#include "IMouseAction.h"
#include "World.h"

namespace physics
{
    
class PolygonDrawer : public IMouseAction
{
public:
    /**
     * @param draw_finish_distance The maximum distance from the first vertex to the latest vertex
     *                             required to finish drawing and try to create a rigidbody.
     *                             Any point within this radius from the first vertex is considered the last vertex.
     */
    PolygonDrawer(std::shared_ptr<World> world, float draw_finish_distance = 20.0f);

    virtual std::string Description() const override;
    virtual std::string Tooltip() const override;

    virtual void OnMouseClick(const Vec3& mouse_pos) override;
    virtual void OnMouseDown(const Vec3& mouse_pos) override;
    virtual void OnMouseRelease(const Vec3& mouse_pos) override;

    const std::vector<Vec3>& CurrentVertices() const;

private:
    bool IsDrawingFinished(const Vec3& clicked_pos) const;
    void TrySpawnObject();
    void AppendVertex(const Vec3& vertex);
    void ClearVertices();

    std::shared_ptr<World> m_world;
    float m_draw_finish_distance;

    // List of all clicked points, which will construct a polygon.
    std::vector<Vec3> m_vertices;

    // Used to calculate the center of the polygon.
    // This is equal to the sum of elements in m_vertices.
    Vec3 m_vertex_sum;
};

} // namespace physics


#endif // PHYSICS_POLYGON_DRAWER_H
