#ifndef TRACEABLESCENE_HPP_
#define TRACEABLESCENE_HPP_

#include "integrators/Integrator.hpp"

#include "primitives/InfiniteSphere.hpp"
#include "primitives/EmbreeUtil.hpp"
#include "primitives/Primitive.hpp"

#include "textures/ConstantTexture.hpp"

#include "cameras/Camera.hpp"

#include "media/Medium.hpp"

#include "RendererSettings.hpp"
#include <vector>
#include <memory>

#include <embree4/rtcore.h>
#include <embree4/rtcore_ray.h>
#include <embree4/rtcore_scene.h>
#include <embree4/rtcore_geometry.h>

namespace Tungsten {

class TraceableScene
{
public:
    struct IntersectionContext
    {
        RTCRayQueryContext rtc;
        IntersectionTemporary *data;
        Ray *ray;
    };

private:
    const float DefaultEpsilon = 5e-4f;

    Camera &_cam;
    Integrator &_integrator;
    std::vector<std::shared_ptr<Primitive>> &_primitives;
    std::vector<std::shared_ptr<Bsdf>> &_bsdfs;
    std::vector<std::shared_ptr<Medium>> &_media;
    std::vector<std::shared_ptr<Primitive>> _lights;
    std::vector<std::shared_ptr<Primitive>> _infiniteLights;
    std::vector<const Primitive *> _finites;
    RendererSettings _settings;

    RTCScene _scene = nullptr;
    RTCGeometry _userGeom = nullptr;

    Box3f _sceneBounds;

public:
    TraceableScene(Camera &cam, Integrator &integrator,
            std::vector<std::shared_ptr<Primitive>> &primitives,
            std::vector<std::shared_ptr<Bsdf>> &bsdfs,
            std::vector<std::shared_ptr<Medium>> &media,
            RendererSettings settings,
            uint32 seed)
    : _cam(cam),
      _integrator(integrator),
      _primitives(primitives),
      _bsdfs(bsdfs),
      _media(media),
      _settings(settings)
    {
        _cam.prepareForRender();
        _cam.requestOutputBuffers(_settings.renderOutputs());

        for (std::shared_ptr<Medium> &m : _media)
            m->prepareForRender();

        for (std::shared_ptr<Bsdf> &b : _bsdfs)
            b->prepareForRender();

        int finiteCount = 0, lightCount = 0;
        for (std::shared_ptr<Primitive> &m : _primitives) {
            m->prepareForRender();
            for (int i = 0; i < m->numBsdfs(); ++i)
                if (m->bsdf(i)->unnamed())
                    m->bsdf(i)->prepareForRender();

            if (!m->isDirac() && !m->isInfinite())
                finiteCount++;

            if (m->isEmissive()) {
                lightCount++;
                if (m->isSamplable())
                    _lights.push_back(m);
                if (m->isInfinite())
                    _infiniteLights.push_back(m);
            }
        }
        if (lightCount == 0) {
            std::shared_ptr<InfiniteSphere> defaultLight = std::make_shared<InfiniteSphere>();
            defaultLight->setEmission(std::make_shared<ConstantTexture>(1.0f));
            _lights.push_back(defaultLight);
            _infiniteLights.push_back(defaultLight);
        }

        for (std::shared_ptr<Primitive> &m : _primitives) {
            if (m->isInfinite() || m->isDirac())
                continue;

            _sceneBounds.grow(m->bounds());
            _finites.push_back(m.get());
        }

        if (_settings.useSceneBvh()) {
            _scene = rtcNewScene(EmbreeUtil::getDevice());
            _userGeom = rtcNewGeometry(EmbreeUtil::getDevice(), RTC_GEOMETRY_TYPE_USER);
            rtcSetGeometryUserPrimitiveCount(_userGeom, _finites.size());
            rtcSetGeometryUserData(_userGeom, this);

            rtcSetGeometryBoundsFunction(_userGeom, [](const RTCBoundsFunctionArguments *args) {
                auto *scene = static_cast<TraceableScene *>(args->geometryUserPtr);
                *args->bounds_o = EmbreeUtil::convert(scene->finites()[args->primID]->bounds());
            }, nullptr);
            rtcSetGeometryIntersectFunction(_userGeom, [](const RTCIntersectFunctionNArguments *args) {
                auto *ictx = reinterpret_cast<IntersectionContext *>(args->context);
                auto *scene = static_cast<TraceableScene *>(args->geometryUserPtr);
                unsigned int i = args->primID;
                RTCRayHit *rh = reinterpret_cast<RTCRayHit *>(args->rayhit);
                if (scene->finites()[i]->intersect(*ictx->ray, *ictx->data)) {
                    rh->ray.tfar = ictx->ray->farT();
                    rh->hit.geomID = args->geomID;
                    rh->hit.primID = i;
                }
            });
            rtcSetGeometryOccludedFunction(_userGeom, [](const RTCOccludedFunctionNArguments *args) {
                auto *scene = static_cast<TraceableScene *>(args->geometryUserPtr);
                unsigned int i = args->primID;
                RTCRay *ray = reinterpret_cast<RTCRay *>(args->ray);
                if (scene->finites()[i]->occluded(EmbreeUtil::convert(*ray)))
                    ray->tfar = -1.0f;
            });

            rtcCommitGeometry(_userGeom);
            rtcAttachGeometry(_scene, _userGeom);
            rtcReleaseGeometry(_userGeom);
            _userGeom = nullptr;
            rtcCommitScene(_scene);
        }

        _integrator.prepareForRender(*this, seed);
    }

    ~TraceableScene()
    {
        _integrator.teardownAfterRender();
        _cam.teardownAfterRender();

        for (std::shared_ptr<Medium> &m : _media)
            m->teardownAfterRender();

        for (std::shared_ptr<Bsdf> &b : _bsdfs)
            b->teardownAfterRender();

        for (std::shared_ptr<Primitive> &m : _primitives) {
            m->teardownAfterRender();
            for (int i = 0; i < m->numBsdfs(); ++i)
                if (m->bsdf(i)->unnamed())
                    m->bsdf(i)->teardownAfterRender();
        }

        if (_scene)
            rtcReleaseScene(_scene);
        _scene = nullptr;
    }

    float hitDistance(Ray &ray) const
    {
        IntersectionTemporary data;
        RTCRayHit rh(EmbreeUtil::convertToRayHit(ray));
        IntersectionContext ictx;
        rtcInitRayQueryContext(&ictx.rtc);
        ictx.data = &data;
        ictx.ray = &ray;
        RTCIntersectArguments args;
        rtcInitIntersectArguments(&args);
        args.context = &ictx.rtc;
        rtcIntersect1(_scene, &rh, &args);
        return ray.farT();
    }

    bool intersect(Ray &ray, IntersectionTemporary &data, IntersectionInfo &info) const
    {
        info.primitive = nullptr;
        data.primitive = nullptr;

        if (_settings.useSceneBvh()) {
            RTCRayHit rh(EmbreeUtil::convertToRayHit(ray));
            IntersectionContext ictx;
            rtcInitRayQueryContext(&ictx.rtc);
            ictx.data = &data;
            ictx.ray = &ray;
            RTCIntersectArguments args;
            rtcInitIntersectArguments(&args);
            args.context = &ictx.rtc;
            rtcIntersect1(_scene, &rh, &args);
        } else {
            for (const Primitive *prim : _finites)
                prim->intersect(ray, data);
        }

        if (data.primitive) {
            info.p = ray.pos() + ray.dir()*ray.farT();
            info.w = ray.dir();
            info.epsilon = DefaultEpsilon;
            data.primitive->intersectionInfo(data, info);
            return true;
        } else {
            return false;
        }
    }

    bool intersectInfinites(Ray &ray, IntersectionTemporary &data, IntersectionInfo &info) const
    {
        info.primitive = nullptr;
        data.primitive = nullptr;

        for (const std::shared_ptr<Primitive> &p : _infiniteLights)
            p->intersect(ray, data);

        if (data.primitive) {
            info.w = ray.dir();
            data.primitive->intersectionInfo(data, info);
            return true;
        } else {
            return false;
        }
    }

    bool occluded(const Ray &ray) const
    {
        if (_settings.useSceneBvh()) {
            RTCRay eRay = EmbreeUtil::convert(ray);
            RTCOccludedArguments args;
            rtcInitOccludedArguments(&args);
            rtcOccluded1(_scene, &eRay, &args);
            return eRay.tfar < 0.0f;
        } else {
            for (const Primitive *prim : _finites)
                if (prim->occluded(ray))
                    return true;
            return false;
        }
    }

    const Box3f &bounds() const
    {
        return _sceneBounds;
    }

    Camera &cam() const
    {
        return _cam;
    }

    Integrator &integrator() const
    {
        return _integrator;
    }

    const std::vector<std::shared_ptr<Primitive>> &primitives() const
    {
        return _primitives;
    }

    std::vector<std::shared_ptr<Primitive>> &lights()
    {
        return _lights;
    }

    const std::vector<std::shared_ptr<Primitive>> &lights() const
    {
        return _lights;
    }

    const std::vector<const Primitive *> &finites() const
    {
        return _finites;
    }

    const std::vector<std::shared_ptr<Medium>> &media() const
    {
        return _media;
    }

    RendererSettings rendererSettings() const
    {
        return _settings;
    }

    RTCScene scene() const
    {
        return _scene;
    }
};

}

#endif /* TRACEABLESCENE_HPP_ */
