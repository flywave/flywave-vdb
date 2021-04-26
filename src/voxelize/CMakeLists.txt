
include_directories(${FLYWAVE_INCLUDE_DIRS})

set(voxelize_SOURCE_FILES barycentric.cc foundry.cc micronizer.cc sampler.cc voxel_pot.cc texture2d.cc)
set(voxelize_HEADER_FILES barycentric.hh bbox.hh color_extract.hh texture2d.hh foundry.hh ray.hh tolerance.hh zero.hh index.hh mesh_adapter.hh micronizer.hh pixel.hh projection.hh rasterize.hh resolution.hh sampler.hh st_policy.hh trees.hh triangle.hh types.hh voxel_pot.hh xparam.hh)

ADD_LIBRARY(voxelize STATIC
            ${voxelize_HEADER_FILES}
            ${voxelize_SOURCE_FILES}
          )

TARGET_INCLUDE_DIRECTORIES(voxelize PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>)

SOURCE_GROUP("src" FILES ${voxelize_SOURCE_FILES})
SOURCE_GROUP("include" FILES ${voxelize_HEADER_FILES})

SET_TARGET_PROPERTIES(voxelize
                    PROPERTIES PUBLIC_HEADER "${headers}")
