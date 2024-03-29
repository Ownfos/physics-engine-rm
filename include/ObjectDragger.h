#ifndef PHYSICS_OBJECT_DRAGGER_H
#define PHYSICS_OBJECT_DRAGGER_H

#include "IMouseAction.h"
#include "World.h"

namespace physics
{

/**
 * @brief Handles picking an object in the scene with mouse
 *        and applying force towards the cursor.
 */
class ObjectDragger : public IMouseAction
{
public:
    ObjectDragger(std::shared_ptr<World> world);

    virtual std::string Description() const override;
    virtual std::string Tooltip() const override;

    /**
     * @brief Try to pick an object under the cursor.
     * 
     * @note Static objects are ignored.
     */
    virtual void OnMouseClick(const Vec3& mouse_pos) override;

    /**
     * @brief Update drag vector, which will be the direction of force.
     */
    virtual void OnMouseDown(const Vec3& mouse_pos) override;

    /**
     * @brief Reset picked object to null.
     */
    virtual void OnMouseRelease(const Vec3& mouse_pos) override;

    /**
     * @brief Pull the selected object towards the cursor.
     * 
     * @warning This should be called only when IsObjectSelected() is true.
     */
    void ApplyDraggingForce(float drag_strength, float time_step);

    /**
     * @brief Test if an object is currently being dragged towards the cursor.
     */
    bool IsObjectSelected() const;

    /**
     * @return The global coordinate of the point where dragging started.
     * 
     * @warning This should be called only when IsObjectSelected() is true.
     */
    Vec3 PickedPoint() const;
    
    /**
     * @return The directional vector from PickedPoint() to current mouse position.
     * 
     * @warning This should be called only when IsObjectSelected() is true.
     */
    Vec3 DragVector() const;

private:
    std::shared_ptr<World> m_world;
    std::shared_ptr<Rigidbody> m_picked_object;
    Vec3 m_picked_offset;
    Vec3 m_drag_vector;
};

} // namespace physics



#endif // PHYSICS_OBJECT_DRAGGER_H
