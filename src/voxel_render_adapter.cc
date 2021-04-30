#include "voxel_render_adapter.hh"

#include <grids/VdbRaymarcher.hpp>

#include <sampling/PathSampleGenerator.hpp>

#include <math/BitManip.hpp>

#include <io/JsonObject.hpp>
#include <io/Scene.hpp>

#include <Debug.hpp>

#include <media/VoxelMedium.hpp>

#include <openvdb/tools/Interpolation.h>

namespace flywave {

std::string voxel_pixel_grid::sampleMethodToString(SampleMethod method) {
  switch (method) {
  default:
  case SampleMethod::ExactNearest:
    return "exact_nearest";
  case SampleMethod::ExactLinear:
    return "exact_linear";
  case SampleMethod::Raymarching:
    return "raymarching";
  }
}

std::string
voxel_pixel_grid::integrationMethodToString(IntegrationMethod method) {
  switch (method) {
  default:
  case IntegrationMethod::ExactNearest:
    return "exact_nearest";
  case IntegrationMethod::ExactLinear:
    return "exact_linear";
  case IntegrationMethod::Raymarching:
    return "raymarching";
  case IntegrationMethod::ResidualRatio:
    return "residual_ratio";
  }
}

voxel_pixel_grid::SampleMethod
voxel_pixel_grid::stringToSampleMethod(const std::string &name) {
  if (name == "exact_nearest")
    return SampleMethod::ExactNearest;
  else if (name == "exact_linear")
    return SampleMethod::ExactLinear;
  else if (name == "raymarching")
    return SampleMethod::Raymarching;
  FAIL("Invalid sample method: '%s'", name);
}

voxel_pixel_grid::IntegrationMethod
voxel_pixel_grid::stringToIntegrationMethod(const std::string &name) {
  if (name == "exact_nearest")
    return IntegrationMethod::ExactNearest;
  else if (name == "exact_linear")
    return IntegrationMethod::ExactLinear;
  else if (name == "raymarching")
    return IntegrationMethod::Raymarching;
  else if (name == "residual_ratio")
    return IntegrationMethod::ResidualRatio;
  FAIL("Invalid integration method: '%s'", name);
}

voxel_pixel_grid::voxel_pixel_grid()
    : _densityName("density"), _emissionName("Cd"),
      _integrationString("exact_nearest"), _sampleString("exact_nearest"),
      _stepSize(5.0f), _densityScale(1.0f), _emissionScale(1.0f),
      _scaleEmissionByDensity(true), _normalizeSize(true),
      _supergridSubsample(10) {
  _integrationMethod = stringToIntegrationMethod(_integrationString);
  _sampleMethod = stringToSampleMethod(_sampleString);
}

inline int roundDown(int a, int b) {
  int c = a >> 31;
  return c ^ ((c ^ a) / b);
}

void voxel_pixel_grid::generateSuperGrid() {
  const int offset = _supergridSubsample / 2;
  auto divideCoord = [&](const openvdb::Coord &a) {
    return openvdb::Coord(roundDown(a.x() + offset, _supergridSubsample),
                          roundDown(a.y() + offset, _supergridSubsample),
                          roundDown(a.z() + offset, _supergridSubsample));
  };

  _superGrid = Vec2fGrid::create(openvdb::Vec2s(0.0f));
  auto accessor = _superGrid->getAccessor();

  Vec2fGrid::Ptr minMaxGrid = Vec2fGrid::create(openvdb::Vec2s(1e30f, 0.0f));
  auto minMaxAccessor = minMaxGrid->getAccessor();

  for (float_grid::ValueOnCIter iter =
           _voxelPixel->get_voxel_grid()->cbeginValueOn();
       iter.test(); ++iter) {
    openvdb::Coord coord = divideCoord(iter.getCoord());
    float d = *iter;
    accessor.setValue(coord,
                      openvdb::Vec2s(accessor.getValue(coord).x() + d, 0.0f));

    openvdb::Vec2s minMax = minMaxAccessor.getValue(coord);
    minMaxAccessor.setValue(coord, openvdb::Vec2s(std::min(minMax.x(), d),
                                                  std::max(minMax.y(), d)));
  }

  float normalize = 1.0f / Tungsten::cube(_supergridSubsample);
  const float Gamma = 2.0f;
  const float D = std::sqrt(3.0f) * _supergridSubsample;
  for (Vec2fGrid::ValueOnIter iter = _superGrid->beginValueOn(); iter.test();
       ++iter) {
    openvdb::Vec2s minMax = minMaxAccessor.getValue(iter.getCoord());

    float muMin = minMax.x();
    float muMax = minMax.y();
    float muAvg = iter->x() * normalize;
    float muR = muMax - muMin;
    float muC = Tungsten::clamp(
        muMin + muR * (std::pow(Gamma, 1.0f / (D * muR)) - 1.0f), muMin, muAvg);
    iter.setValue(openvdb::Vec2s(muC, 0.0f));
  }

  for (pixel_grid::ValueOnCIter iter =
           _voxelPixel->get_pixel_grid()->cbeginValueOn();
       iter.test(); ++iter) {
    openvdb::Coord coord = divideCoord(iter.getCoord());
    openvdb::Vec2s v = accessor.getValue(coord);
    float residual = std::max<float>(v.y(), std::abs(*iter - v.x()));
    accessor.setValue(coord, openvdb::Vec2s(v.x(), residual));
  }
}

void voxel_pixel_grid::fromJson(Tungsten::JsonPtr value,
                                const Tungsten::Scene &scene) {
  if (auto path = value["file"])
    _path = scene.fetchResource(path);
  value.getField("grid_name",
                 _densityName); /* Deprecated field for density grid name */
  value.getField("density_name", _densityName);
  value.getField("density_scale", _densityScale);
  value.getField("emission_name", _emissionName);
  value.getField("emission_scale", _emissionScale);
  value.getField("scale_emission_by_density", _scaleEmissionByDensity);
  value.getField("normalize_size", _normalizeSize);
  value.getField("integration_method", _integrationString);
  value.getField("sampling_method", _sampleString);
  value.getField("step_size", _stepSize);
  value.getField("supergrid_subsample", _supergridSubsample);
  value.getField("transform", _configTransform);

  _integrationMethod = stringToIntegrationMethod(_integrationString);
  _sampleMethod = stringToSampleMethod(_sampleString);
}

rapidjson::Value voxel_pixel_grid::toJson(Allocator &allocator) const {
  Tungsten::JsonObject result{Grid::toJson(allocator),
                              allocator,
                              "type",
                              "vdb",
                              "file",
                              *_path,
                              "density_name",
                              _densityName,
                              "density_scale",
                              _densityScale,
                              "emission_name",
                              _emissionName,
                              "emission_scale",
                              _emissionScale,
                              "scale_emission_by_density",
                              _scaleEmissionByDensity,
                              "normalize_size",
                              _normalizeSize,
                              "integration_method",
                              _integrationString,
                              "sampling_method",
                              _sampleString,
                              "transform",
                              _configTransform};
  if (_integrationMethod == IntegrationMethod::ResidualRatio)
    result.add("supergrid_subsample", _supergridSubsample);
  if (_integrationMethod == IntegrationMethod::Raymarching ||
      _sampleMethod == SampleMethod::Raymarching)
    result.add("step_size", _stepSize);

  return result;
}

void voxel_pixel_grid::loadResources() {
  _voxelPixel = std::make_shared<flywave::voxel_pixel>();
  _voxelPixel->read(_path->absolute().asString());

  auto accessor = _voxelPixel->get_voxel_grid()->getAccessor();
  for (float_grid::ValueOnIter iter =
           _voxelPixel->get_voxel_grid()->beginValueOn();
       iter.test(); ++iter)
    iter.setValue((*iter) * _densityScale);

  Tungsten::Vec3d densityCenter(_voxelPixel->get_voxel_grid()
                                    ->transform()
                                    .indexToWorld(openvdb::Vec3d(0, 0, 0))
                                    .asPointer());
  Tungsten::Vec3d densitySpacing(_voxelPixel->get_voxel_grid()
                                     ->transform()
                                     .indexToWorld(openvdb::Vec3d(1, 1, 1))
                                     .asPointer());
  densitySpacing -= densityCenter;

  Tungsten::Vec3d emissionCenter, emissionSpacing;
  if (_voxelPixel) {
    emissionCenter = Tungsten::Vec3d(_voxelPixel->get_pixel_grid()
                                         ->transform()
                                         .indexToWorld(openvdb::Vec3d(0, 0, 0))
                                         .asPointer());
    emissionSpacing = Tungsten::Vec3d(_voxelPixel->get_pixel_grid()
                                          ->transform()
                                          .indexToWorld(openvdb::Vec3d(1, 1, 1))
                                          .asPointer());
    emissionSpacing -= emissionCenter;
  } else {
    emissionCenter = densityCenter;
    emissionSpacing = densitySpacing;
  }
  _emissionIndexOffset =
      Tungsten::Vec3f((densityCenter - emissionCenter) / emissionSpacing);

  openvdb::CoordBBox bbox =
      _voxelPixel->get_voxel_grid()->evalActiveVoxelBoundingBox();
  Tungsten::Vec3i minP =
      Tungsten::Vec3i(bbox.min().x(), bbox.min().y(), bbox.min().z());
  Tungsten::Vec3i maxP =
      Tungsten::Vec3i(bbox.max().x(), bbox.max().y(), bbox.max().z()) + 1;
  Tungsten::Vec3f diag = Tungsten::Vec3f(maxP - minP);

  float scale;
  Tungsten::Vec3f center;
  if (_normalizeSize) {
    scale = 1.0f / diag.max();
    diag *= scale;
    center = Tungsten::Vec3f(minP) * scale +
             Tungsten::Vec3f(diag.x(), 0.0f, diag.z()) * 0.5f;
  } else {
    scale = densitySpacing.min();
    center = -Tungsten::Vec3f(densityCenter);
  }

  if (_integrationMethod == IntegrationMethod::ResidualRatio)
    generateSuperGrid();

  _transform = Tungsten::Mat4f::translate(-center) *
               Tungsten::Mat4f::scale(Tungsten::Vec3f(scale));
  _invTransform = Tungsten::Mat4f::scale(Tungsten::Vec3f(1.0f / scale)) *
                  Tungsten::Mat4f::translate(center);
  _bounds = Tungsten::Box3f(Tungsten::Vec3f(minP), Tungsten::Vec3f(maxP));

  if (_sampleMethod == SampleMethod::ExactLinear ||
      _integrationMethod == IntegrationMethod::ExactLinear) {
    auto accessor = _voxelPixel->get_voxel_grid()->getAccessor();
    for (float_grid::ValueOnCIter iter =
             _voxelPixel->get_voxel_grid()->cbeginValueOn();
         iter.test(); ++iter) {
      if (*iter != 0.0f)
        for (int z = -1; z <= 1; ++z)
          for (int y = -1; y <= 1; ++y)
            for (int x = -1; x <= 1; ++x)
              accessor.setValueOn(iter.getCoord() + openvdb::Coord(x, y, z));
      _bounds =
          Tungsten::Box3f(Tungsten::Vec3f(minP - 1), Tungsten::Vec3f(maxP + 1));
    }
  }

  _invConfigTransform = _configTransform.invert();
}

Tungsten::Mat4f voxel_pixel_grid::naturalTransform() const {
  return _configTransform * _transform;
}

Tungsten::Mat4f voxel_pixel_grid::invNaturalTransform() const {
  return _invTransform * _invConfigTransform;
}

Tungsten::Box3f voxel_pixel_grid::bounds() const { return _bounds; }

template <typename TreeT>
static inline float gridAt(TreeT &acc, Tungsten::Vec3f p) {
  return openvdb::tools::BoxSampler::sample(
      acc, openvdb::Vec3R(p.x(), p.y(), p.z()));
}

float voxel_pixel_grid::density(Tungsten::Vec3f p) const {
  return gridAt(_voxelPixel->get_voxel_grid()->tree(), p);
}

Tungsten::Vec3f voxel_pixel_grid::emission(Tungsten::Vec3f p) const {
  if (_voxelPixel) {
    Tungsten::Vec3f op = p + _emissionIndexOffset;
    auto pix = openvdb::tools::BoxSampler::sample(
        _voxelPixel->get_pixel_grid()->tree(),
        openvdb::Vec3R(op.x(), op.y(), op.z()));
    Tungsten::Vec3f result =
        _emissionScale * Tungsten::Vec3f(pix._data._color.x() / 255,
                                         pix._data._color.y() / 255,
                                         pix._data._color.z() / 255);
    if (_scaleEmissionByDensity)
      result *= density(p);
    return result;
  } else {
    return Tungsten::Vec3f(0.0f);
  }
}

float voxel_pixel_grid::opticalDepth(Tungsten::PathSampleGenerator &sampler,
                                     Tungsten::Vec3f p, Tungsten::Vec3f w,
                                     float t0, float t1) const {
  auto accessor = _voxelPixel->get_voxel_grid()->getConstAccessor();

  if (_integrationMethod == IntegrationMethod::ExactNearest) {
    Tungsten::VdbRaymarcher<float_grid::TreeType, 3> dda;

    float integral = 0.0f;
    dda.march(Tungsten::DdaRay(p + 0.5f, w), t0, t1, accessor,
              [&](openvdb::Coord voxel, float ta, float tb) {
                integral += accessor.getValue(voxel) * (tb - ta);
                return false;
              });
    return integral;
  } else if (_integrationMethod == IntegrationMethod::ExactLinear) {
    Tungsten::VdbRaymarcher<float_grid::TreeType, 3> dda;

    float integral = 0.0f;
    float fa = gridAt(accessor, p + w * t0);
    dda.march(Tungsten::DdaRay(p, w), t0, t1, accessor,
              [&](openvdb::Coord /*voxel*/, float ta, float tb) {
                float fb = gridAt(accessor, p + w * tb);
                integral += (fa + fb) * 0.5f * (tb - ta);
                fa = fb;
                return false;
              });
    return integral;
  } else if (_integrationMethod == IntegrationMethod::ResidualRatio) {
    Tungsten::VdbRaymarcher<Vec2fGrid::TreeType, 3> dda;

    float scale = _supergridSubsample;
    float invScale = 1.0f / scale;

    auto superAccessor = _superGrid->getConstAccessor();

    Tungsten::UniformSampler &generator = sampler.uniformGenerator();

    float controlIntegral = 0.0f;
    float Tr = 1.0f;
    dda.march(
        Tungsten::DdaRay(p * invScale + 0.5f, w), t0 * invScale, t1 * invScale,
        superAccessor, [&](openvdb::Coord voxel, float ta, float tb) {
          openvdb::Vec2s v = superAccessor.getValue(voxel);
          float muC = v.x();
          float muR = v.y();
          muR *= scale;

          controlIntegral += muC * (tb - ta);

          while (true) {
            ta -= Tungsten::BitManip::normalizedLog(generator.nextI()) / muR;
            if (ta >= tb)
              break;
            Tr *= 1.0f -
                  scale * ((gridAt(accessor, p + w * ta * scale) - muC) / muR);
          }

          return false;
        });
    return controlIntegral - std::log(Tr);
  } else {
    float ta = t0;
    float fa = gridAt(accessor, p + w * t0);
    float integral = 0.0f;
    float dT = sampler.next1D() * _stepSize;
    do {
      float tb = std::min(ta + dT, t1);
      float fb = gridAt(accessor, p + w * tb);
      integral += (fa + fb) * 0.5f * (tb - ta);
      ta = tb;
      fa = fb;
      dT = _stepSize;
    } while (ta < t1);
    return integral;
  }
}

Tungsten::Vec2f
voxel_pixel_grid::inverseOpticalDepth(Tungsten::PathSampleGenerator &sampler,
                                      Tungsten::Vec3f p, Tungsten::Vec3f w,
                                      float t0, float t1, float tau) const {
  auto accessor = _voxelPixel->get_voxel_grid()->getConstAccessor();

  if (_sampleMethod == SampleMethod::ExactNearest) {
    Tungsten::VdbRaymarcher<openvdb::FloatGrid::TreeType, 3> dda;

    float integral = 0.0f;
    Tungsten::Vec2f result(t1, 0.0f);
    bool exited =
        !dda.march(Tungsten::DdaRay(p + 0.5f, w), t0, t1, accessor,
                   [&](openvdb::Coord voxel, float ta, float tb) {
                     float v = accessor.getValue(voxel);
                     float delta = v * (tb - ta);
                     if (integral + delta >= tau) {
                       result = Tungsten::Vec2f(
                           ta + (tb - ta) * (tau - integral) / delta, v);
                       return true;
                     }
                     integral += delta;
                     return false;
                   });
    return exited ? Tungsten::Vec2f(t1, integral) : result;
  } else if (_sampleMethod == SampleMethod::ExactLinear) {
    Tungsten::VdbRaymarcher<openvdb::FloatGrid::TreeType, 3> dda;

    float integral = 0.0f;
    float fa = gridAt(accessor, p + w * t0);
    Tungsten::Vec2f result(t1, 0.0f);
    bool exited = !dda.march(
        Tungsten::DdaRay(p + 0.5f, w), t0, t1, accessor,
        [&](openvdb::Coord /*voxel*/, float ta, float tb) {
          float fb = gridAt(accessor, p + tb * w);
          float delta = (fb + fa) * 0.5f * (tb - ta);
          if (integral + delta >= tau) {
            float a = (fb - fa);
            float b = fa;
            float c = (integral - tau) / (tb - ta);
            float x1;
            if (std::abs(a) < 1e-6f) {
              x1 = -c / b;
            } else {
              float mantissa = std::max(b * b - 2.0f * a * c, 0.0f);
              x1 = (-b + std::sqrt(mantissa)) / a;
            }
            x1 = Tungsten::clamp(x1, 0.0f, 1.0f);
            result = Tungsten::Vec2f(ta + (tb - ta) * x1, fa + (fb - fa) * x1);
            return true;
          }
          integral += delta;
          fa = fb;
          return false;
        });
    return exited ? Tungsten::Vec2f(t1, integral) : result;
  } else {
    float ta = t0;
    float fa = gridAt(accessor, p + w * t0);
    float integral = 0.0f;
    float dT = sampler.next1D() * _stepSize;
    do {
      float tb = std::min(ta + dT, t1);
      float fb = gridAt(accessor, p + w * tb);
      float delta = (fa + fb) * 0.5f * (tb - ta);
      if (integral + delta >= tau) {
        float a = (fb - fa);
        float b = fa;
        float c = (integral - tau) / (tb - ta);
        float mantissa = std::max(b * b - 2.0f * a * c, 0.0f);
        float x1 = (-b + std::sqrt(mantissa)) / a;
        return Tungsten::Vec2f(ta + (tb - ta) * x1, fa + (fb - fa) * x1);
      }
      integral += delta;
      ta = tb;
      fa = fb;
      dT = _stepSize;
    } while (ta < t1);
    return Tungsten::Vec2f(t1, integral);
  }
}

} // namespace flywave