#include "Spring.h"

namespace physics
{

Vec3 AnchorPoint::GlobalPosition() const
{
    return object->Transform().GlobalPosition(local_pos);
}

void AnchorPoint::ApplyImpulse(const Vec3& impulse, float delta_time)
{
    const auto impact_point = GlobalPosition() - object->Transform().Position();
    object->ApplyImpulse(impact_point, impulse, delta_time);
}

void Spring::ApplyImpulse(float delta_time)
{
    // Calculate how far we are from the neutral position.
    const auto displacement = end.GlobalPosition() - start.GlobalPosition();
    const auto offset_from_neutral = displacement.Magnitude() - neutral_distance;

    // Construct impulse using Hooke's law.
    auto impulse = displacement;
    impulse.Normalize();
    impulse *= offset_from_neutral * coefficient * delta_time;

    start.ApplyImpulse(impulse, delta_time);
    end.ApplyImpulse(-impulse, delta_time);
}

} // namespace physics
