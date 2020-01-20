#include "pose2d.h"

#include "processing/tools/rotation.h"

namespace lma
{
////////////////////////////////////////////////////////////////////////////////////////////////////
// On Poses
////////////////////////////////////////////////////////////////////////////////////////////////////
void apply_increment(Pose2D& p, const double delta[Size<Pose2D>::value], const Adl&)
{
    p.translation()[0] += delta[0];
    p.translation()[1] += delta[1];

    apply_rotation(p.rotation(), delta[2]);
}

void apply_small_increment(Pose2D& p, double h, const v::core::numeric_tag<0>&, const Adl&)
{
    p.translation()[0] += h;
}

void apply_small_increment(Pose2D& p, double h, const v::core::numeric_tag<1>&, const Adl&)
{
    p.translation()[1] += h;
}

void apply_small_increment(Pose2D& p, double h, const v::core::numeric_tag<2>&, const Adl&)
{
    apply_rotation(p.rotation(), h);
}

} // namespace lma
