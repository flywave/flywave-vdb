# Copyright Contributors to the OpenVDB Project
# SPDX-License-Identifier: MPL-2.0
#
#[=======================================================================[

  CMake Configuration for OpenVDB Core

#]=======================================================================]

cmake_minimum_required(VERSION 3.12)
project(OpenVDBCore LANGUAGES CXX)

include(GNUInstallDirs)

if(UNIX)
  find_package(Threads REQUIRED)
endif()

##########################################################################

##### Core library configuration

# Configure the BuildConfig header

if(USE_IMATH_HALF)
  set(OPENVDB_BUILDCONFIG_USE_IMATH_HALF 1)
endif()

configure_file(BuildConfig.h.in BuildConfig.h)

##########################################################################

set(OPENVDB_LIBRARY_SOURCE_FILES
  Grid.cc
  io/Archive.cc
  io/Compression.cc
  io/DelayedLoadMetadata.cc
  io/File.cc
  io/GridDescriptor.cc
  io/Queue.cc
  io/Stream.cc
  io/TempFile.cc
  math/Half.cc
  math/Maps.cc
  math/Proximity.cc
  math/QuantizedUnitVec.cc
  math/Transform.cc
  Metadata.cc
  MetaMap.cc
  openvdb.cc
  Platform.cc
  points/AttributeArray.cc
  points/AttributeArrayString.cc
  points/AttributeGroup.cc
  points/AttributeSet.cc
  points/StreamCompression.cc
  points/points.cc
  util/Formats.cc
  util/Util.cc
)

set(OPENVDB_LIBRARY_INCLUDE_FILES
  Exceptions.h
  Grid.h
  Metadata.h
  MetaMap.h
  openvdb.h
  Platform.h
  PlatformConfig.h
  Types.h
  TypeList.h
  version.h
)

set(OPENVDB_LIBRARY_IO_INCLUDE_FILES
  io/Archive.h
  io/Compression.h
  io/DelayedLoadMetadata.h
  io/File.h
  io/GridDescriptor.h
  io/io.h
  io/Queue.h
  io/Stream.h
  io/TempFile.h
)

set(OPENVDB_LIBRARY_MATH_INCLUDE_FILES
  math/BBox.h
  math/ConjGradient.h
  math/Coord.h
  math/DDA.h
  math/FiniteDifference.h
  math/Half.h
  math/HalfLimits.h
  math/LegacyFrustum.h
  math/Maps.h
  math/Mat.h
  math/Mat3.h
  math/Mat4.h
  math/Math.h
  math/Operators.h
  math/Proximity.h
  math/QuantizedUnitVec.h
  math/Quat.h
  math/Ray.h
  math/Stats.h
  math/Stencils.h
  math/Transform.h
  math/Tuple.h
  math/Vec2.h
  math/Vec3.h
  math/Vec4.h
)

set(OPENVDB_LIBRARY_POINTS_INCLUDE_FILES
  points/AttributeArray.h
  points/AttributeArrayString.h
  points/AttributeGroup.h
  points/AttributeSet.h
  points/IndexFilter.h
  points/IndexIterator.h
  points/PointAdvect.h
  points/PointAttribute.h
  points/PointConversion.h
  points/PointCount.h
  points/PointDataGrid.h
  points/PointDelete.h
  points/PointGroup.h
  points/PointMask.h
  points/PointMove.h
  points/PointSample.h
  points/PointScatter.h
  points/StreamCompression.h
)

set(OPENVDB_LIBRARY_TOOLS_INCLUDE_FILES
  tools/Activate.h
  tools/ChangeBackground.h
  tools/Clip.h
  tools/Composite.h
  tools/Dense.h
  tools/DenseSparseTools.h
  tools/Diagnostics.h
  tools/FastSweeping.h
  tools/Filter.h
  tools/FindActiveValues.h
  tools/GridOperators.h
  tools/GridTransformer.h
  tools/Interpolation.h
  tools/LevelSetAdvect.h
  tools/LevelSetFilter.h
  tools/LevelSetFracture.h
  tools/LevelSetMeasure.h
  tools/LevelSetMorph.h
  tools/LevelSetPlatonic.h
  tools/LevelSetRebuild.h
  tools/LevelSetSphere.h
  tools/LevelSetTracker.h
  tools/LevelSetUtil.h
  tools/Mask.h
  tools/Merge.h
  tools/MeshToVolume.h
  tools/Morphology.h
  tools/MultiResGrid.h
  tools/NodeVisitor.h
  tools/ParticleAtlas.h
  tools/ParticlesToLevelSet.h
  tools/PointAdvect.h
  tools/PointIndexGrid.h
  tools/PointPartitioner.h
  tools/PointScatter.h
  tools/PointsToMask.h
  tools/PoissonSolver.h
  tools/PotentialFlow.h
  tools/Prune.h
  tools/RayIntersector.h
  tools/RayTracer.h
  tools/SignedFloodFill.h
  tools/Statistics.h
  tools/TopologyToLevelSet.h
  tools/ValueTransformer.h
  tools/VectorTransformer.h
  tools/VelocityFields.h
  tools/VolumeAdvect.h
  tools/VolumeToMesh.h
  tools/VolumeToSpheres.h
)

set(OPENVDB_LIBRARY_TREE_INCLUDE_FILES
  tree/InternalNode.h
  tree/Iterator.h
  tree/LeafBuffer.h
  tree/LeafManager.h
  tree/LeafNode.h
  tree/LeafNodeBool.h
  tree/LeafNodeMask.h
  tree/NodeManager.h
  tree/NodeUnion.h
  tree/RootNode.h
  tree/Tree.h
  tree/TreeIterator.h
  tree/ValueAccessor.h
)

set(OPENVDB_LIBRARY_UTIL_INCLUDE_FILES
  util/CpuTimer.h
  util/Formats.h
  util/logging.h
  util/MapsUtil.h
  util/Name.h
  util/NodeMasks.h
  util/NullInterrupter.h
  util/PagedArray.h
  util/Util.h
)

add_library(openvdb STATIC ${OPENVDB_LIBRARY_SOURCE_FILES})

list(APPEND OPENVDB_CORE_PRIVATE_DEFINES -DOPENVDB_PRIVATE)
if(USE_BLOSC)
  list(APPEND OPENVDB_CORE_PRIVATE_DEFINES -DOPENVDB_USE_BLOSC)
endif()

if(USE_BLOSC OR USE_ZLIB)
  list(APPEND OPENVDB_CORE_PRIVATE_DEFINES -DOPENVDB_USE_ZLIB)
endif()

# Public defines
if(WIN32)
  # @note OPENVDB_OPENEXR_STATICLIB is old functionality from the makefiles
  #       used in PlatformConfig.h to configure EXR exports. Once this file
  #       is completely removed, this define can be too
  if(USE_IMATH_HALF)
    if(ILMBASE_USE_STATIC_LIBS OR (${ILMBASE_Half_LIB_TYPE} STREQUAL STATIC_LIBRARY))
      list(APPEND OPENVDB_CORE_PUBLIC_DEFINES -DOPENVDB_OPENEXR_STATICLIB)
    endif()
  endif()
  list(APPEND OPENVDB_CORE_PUBLIC_DEFINES -D_WIN32 -DNOMINMAX)
endif()

IF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
  SET_TARGET_PROPERTIES(openvdb PROPERTIES FOLDER external)
ENDIF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
