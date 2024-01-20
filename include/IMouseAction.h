#ifndef PHYSICS_I_MOUSE_ACTION_H
#define PHYSICS_I_MOUSE_ACTION_H

#include "Vec3.h"
#include <string>

namespace physics
{

/**
 * @brief An interface for user interactions through mouse clicks.
 */
class IMouseAction
{
public:
    virtual ~IMouseAction() = default;

    virtual std::string Description() const = 0;
    virtual std::string Tooltip() const = 0;

    virtual void OnMouseClick(const Vec3& mouse_pos) = 0;
    virtual void OnMouseDown(const Vec3& mouse_pos) = 0;
    virtual void OnMouseRelease(const Vec3& mouse_pos) = 0;
};

} // namespace physics


#endif // PHYSICS_I_MOUSE_ACTION_H
