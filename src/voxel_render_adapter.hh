#pragma once

#include "voxel_pixel.hh"

#include <grids/Grid.hpp>

#include <io/FileUtils.hpp>

#include <media/Medium.hpp>

namespace flywave {

class voxel_pixel_grid : public Tungsten::Grid {
  enum class IntegrationMethod {
    ExactNearest,
    ExactLinear,
    Raymarching,
    ResidualRatio,
  };
  enum class SampleMethod {
    ExactNearest,
    ExactLinear,
    Raymarching,
  };

  typedef openvdb::tree::Tree4<openvdb::Vec2s, 5, 4, 3>::Type Vec2fTree;
  typedef openvdb::Grid<Vec2fTree> Vec2fGrid;

  Tungsten::PathPtr _path;
  std::string _densityName;
  std::string _emissionName;
  std::string _integrationString;
  std::string _sampleString;
  float _stepSize;
  float _densityScale;
  float _emissionScale;
  bool _scaleEmissionByDensity;
  bool _normalizeSize;
  int _supergridSubsample;
  Tungsten::Mat4f _configTransform;
  Tungsten::Mat4f _invConfigTransform;

  Tungsten::Vec3f _emissionIndexOffset;

  IntegrationMethod _integrationMethod;
  SampleMethod _sampleMethod;
  std::shared_ptr<flywave::voxel_pixel> _voxelPixel;
  Vec2fGrid::Ptr _superGrid;
  Tungsten::Mat4f _transform;
  Tungsten::Mat4f _invTransform;
  Tungsten::Box3f _bounds;

  static std::string sampleMethodToString(SampleMethod method);
  static std::string integrationMethodToString(IntegrationMethod method);

  static SampleMethod stringToSampleMethod(const std::string &name);
  static IntegrationMethod stringToIntegrationMethod(const std::string &name);

  void generateSuperGrid();

public:
  voxel_pixel_grid();

  virtual void fromJson(Tungsten::JsonPtr value,
                        const Tungsten::Scene &scene) override;

  virtual rapidjson::Value toJson(Allocator &allocator) const override;

  virtual void loadResources() override;

  virtual Tungsten::Mat4f naturalTransform() const override;
  virtual Tungsten::Mat4f invNaturalTransform() const override;
  virtual Tungsten::Box3f bounds() const override;

  float density(Tungsten::Vec3f p) const override;
  Tungsten::Vec3f emission(Tungsten::Vec3f p) const override;
  float opticalDepth(Tungsten::PathSampleGenerator &sampler, Tungsten::Vec3f p,
                     Tungsten::Vec3f w, float t0, float t1) const override;
  Tungsten::Vec2f inverseOpticalDepth(Tungsten::PathSampleGenerator &sampler,
                                      Tungsten::Vec3f p, Tungsten::Vec3f w,
                                      float t0, float t1,
                                      float tau) const override;
};

} // namespace flywave
