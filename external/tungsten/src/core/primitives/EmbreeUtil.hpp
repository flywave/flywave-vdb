#ifndef EMBREEUTIL_HPP_
#define EMBREEUTIL_HPP_

#include "math/Ray.hpp"
#include "math/Box.hpp"
#include "math/Mat4f.hpp"

#include <embree4/rtcore.h>
#include <embree4/rtcore_ray.h>

namespace Tungsten {

namespace EmbreeUtil {

void initDevice();
RTCDevice getDevice();

inline RTCBounds convert(const Box3f &b)
{
    return RTCBounds{
        b.min().x(), b.min().y(), b.min().z(), 0.0f,
        b.max().x(), b.max().y(), b.max().z(), 0.0f
    };
}

inline Box3f convert(const RTCBounds &b)
{
    return Box3f(
        Vec3f(b.lower_x, b.lower_y, b.lower_z),
        Vec3f(b.upper_x, b.upper_y, b.upper_z)
    );
}

inline Ray convert(const RTCRay &r)
{
    return Ray(Vec3f(r.org_x, r.org_y, r.org_z), Vec3f(r.dir_x, r.dir_y, r.dir_z), r.tnear, r.tfar);
}

inline RTCRay convert(const Ray &r)
{
    RTCRay ray;
    ray.org_x = r.pos().x();
    ray.org_y = r.pos().y();
    ray.org_z = r.pos().z();
    ray.dir_x = r.dir().x();
    ray.dir_y = r.dir().y();
    ray.dir_z = r.dir().z();
    ray.tnear = r.nearT();
    ray.tfar  = r.farT();
    ray.time  = 0.0f;
    ray.mask  = -1u;
    ray.id    = 0;
    ray.flags = 0;
    return ray;
}

inline RTCRayHit convertToRayHit(const Ray &r)
{
    RTCRayHit rh;
    rh.ray = convert(r);
    rh.hit.Ng_x = 0.0f;
    rh.hit.Ng_y = 0.0f;
    rh.hit.Ng_z = 0.0f;
    rh.hit.u = 0.0f;
    rh.hit.v = 0.0f;
    rh.hit.primID = RTC_INVALID_GEOMETRY_ID;
    rh.hit.geomID = RTC_INVALID_GEOMETRY_ID;
    for (int i = 0; i < RTC_MAX_INSTANCE_LEVEL_COUNT; ++i)
        rh.hit.instID[i] = RTC_INVALID_GEOMETRY_ID;
    return rh;
}

}

}

#endif /* EMBREEUTIL_HPP_ */
