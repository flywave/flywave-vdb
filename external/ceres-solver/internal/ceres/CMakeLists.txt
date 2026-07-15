# Ceres Solver - A fast non-linear least squares minimizer
# Copyright 2022 Google Inc. All rights reserved.
# http://ceres-solver.org/
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form %Cmust reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of Google Inc. nor the names of its contributors may be
#   used to endorse or promote products derived from this software without
#   specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: keir@google.com (Keir Mierle)

# Build the list of dependencies for Ceres based on the current configuration.

# Avoid 'xxx.cc has no symbols' warnings from source files which are 'empty'
# when their enclosing #ifdefs are disabled.
find_package(Threads REQUIRED)
list(APPEND CERES_LIBRARY_PRIVATE_DEPENDENCIES Threads::Threads)
# Make dependency visible to the parent CMakeLists.txt
set(Threads_DEPENDENCY "find_dependency (Threads)" PARENT_SCOPE)

# Source files that contain public symbols and live in the ceres namespaces.
# Such symbols are expected to be marked with CERES_EXPORT and the files below
# sorted in lexicographical order.
set(CERES_EXPORTED_SRCS
    c_api.cc
    gradient_checker.cc
    gradient_problem.cc
    loss_function.cc
    manifold.cc
    normal_prior.cc
    problem.cc
    solver.cc
    types.cc
)

# Also depend on the header files so that they appear in IDEs.
file(GLOB CERES_INTERNAL_HDRS *.h)
if (MINIGLOG)
  file(GLOB MINIGLOG_HDRS miniglog/glog/*.h)
  list(APPEND CERES_INTERNAL_HDRS ${MINIGLOG_HDRS})
  if (ANDROID)
    list(APPEND CERES_LIBRARY_PUBLIC_DEPENDENCIES log)
  endif()
endif()

# Depend also on public headers so they appear in IDEs.
file(GLOB CERES_PUBLIC_HDRS ${Ceres_SOURCE_DIR}/include/ceres/*.h)
file(GLOB CERES_PUBLIC_INTERNAL_HDRS ${Ceres_SOURCE_DIR}/include/ceres/internal/*.h)
file(GLOB CERES_PUBLIC_INTERNAL_HDRS
  ${CMAKE_CURRENT_BINARY_DIR}/include/ceres/internal/*.h)

# Include the specialized schur solvers.
if (SCHUR_SPECIALIZATIONS)
  file(GLOB CERES_INTERNAL_SCHUR_FILES generated/*.cc)
else (SCHUR_SPECIALIZATIONS)
  # Only the fully dynamic solver. The build is much faster this way.
  file(GLOB CERES_INTERNAL_SCHUR_FILES generated/*_d_d_d.cc)
endif (SCHUR_SPECIALIZATIONS)

# The generated specializations of the Schur eliminator include
# schur_eliminator_impl.h which defines EIGEN_CACHEFRIENDLY_PRODUCT_THRESHOLD
# to a different value than Eigen's default.  Depending on the order of files
# in the unity build this can lead to clashes.  Additionally, these files are
# already generated in a way which leads to fairly large compilation units,
# so the gains from a unity build would be marginal.
set_source_files_properties(${CERES_INTERNAL_SCHUR_FILES} PROPERTIES
  SKIP_UNITY_BUILD_INCLUSION ON)

if (NOT MINIGLOG AND GLOG_FOUND)
  list(APPEND CERES_LIBRARY_PUBLIC_DEPENDENCIES ${GLOG_LIBRARIES})
  if (gflags_FOUND)
    # If glog & gflags are both found, we assume that glog was built with
    # gflags, as it is awkward to perform a try_compile() to verify this
    # when gflags is an imported target (as it is in newer versions).
    # As glog #includes gflags/gflags.h in glog/logging.h if compiled with
    # gflags, it is thus a public dependency for Ceres in this case.
    list(APPEND CERES_LIBRARY_PUBLIC_DEPENDENCIES gflags)
  endif()
endif (NOT MINIGLOG AND GLOG_FOUND)

if (SUITESPARSE AND SuiteSparse_FOUND)
  # Define version information for use in Solver::FullReport.
  add_definitions(-DCERES_SUITESPARSE_VERSION="${SuiteSparse_VERSION}")
  list(APPEND CERES_LIBRARY_PRIVATE_DEPENDENCIES SuiteSparse::CHOLMOD
    SuiteSparse::SPQR)

  if (SuiteSparse_Partition_FOUND)
    list(APPEND CERES_LIBRARY_PRIVATE_DEPENDENCIES SuiteSparse::Partition)
  endif (SuiteSparse_Partition_FOUND)
endif (SUITESPARSE AND SuiteSparse_FOUND)

if (SuiteSparse_Partition_FOUND OR EIGENMETIS)
  # Define version information for use in Solver::FullReport.
  add_definitions(-DCERES_METIS_VERSION="${METIS_VERSION}")
endif (SuiteSparse_Partition_FOUND OR EIGENMETIS)

if (ACCELERATESPARSE AND AccelerateSparse_FOUND)
  list(APPEND CERES_LIBRARY_PRIVATE_DEPENDENCIES ${AccelerateSparse_LIBRARIES})
  list(APPEND CERES_LIBRARY_PRIVATE_DEPENDENCIES_INCLUDE_DIRS ${AccelerateSparse_INCLUDE_DIRS})
endif()

if (USE_CUDA)
  list(APPEND CERES_LIBRARY_PRIVATE_DEPENDENCIES
    CUDA::cublas
    CUDA::cudart
    CUDA::cusolver
    CUDA::cusparse)
  set_source_files_properties(cuda_kernels_vector_ops.cu.cc PROPERTIES LANGUAGE CUDA)
  set_source_files_properties(cuda_kernels_bsm_to_crs.cu.cc PROPERTIES LANGUAGE CUDA)
  add_library(ceres_cuda_kernels STATIC cuda_kernels_vector_ops.cu.cc cuda_kernels_bsm_to_crs.cu.cc)
  target_compile_features(ceres_cuda_kernels PRIVATE cxx_std_14)
  target_compile_definitions(ceres_cuda_kernels PRIVATE CERES_STATIC_DEFINE)
  # Enable __host__ / __device__ annotations in lambda declarations
  target_compile_options(ceres_cuda_kernels PRIVATE --extended-lambda)
  target_include_directories(ceres_cuda_kernels PRIVATE  ${Ceres_SOURCE_DIR}/include  ${Ceres_SOURCE_DIR}/internal ${CMAKE_CURRENT_BINARY_DIR}/include)
  list(APPEND CERES_LIBRARY_PRIVATE_DEPENDENCIES ceres_cuda_kernels)
endif (USE_CUDA)

if (LAPACK_FOUND)
  list(APPEND CERES_LIBRARY_PRIVATE_DEPENDENCIES ${LAPACK_LIBRARIES})
endif ()

# Source files that contain private symbols and live in the ceres::internal
# namespace. The corresponding symbols (classes, functions, etc.) are expected
# to be marked with CERES_NO_EXPORT and the files below sorted in
# lexicographical order.
set(CERES_INTERNAL_SRC
    accelerate_sparse.cc
    array_utils.cc
    block_evaluate_preparer.cc
    block_jacobi_preconditioner.cc
    block_jacobian_writer.cc
    block_random_access_dense_matrix.cc
    block_random_access_diagonal_matrix.cc
    block_random_access_matrix.cc
    block_random_access_sparse_matrix.cc
    block_sparse_matrix.cc
    block_structure.cc
    callbacks.cc
    canonical_views_clustering.cc
    cgnr_solver.cc
    compressed_col_sparse_matrix_utils.cc
    compressed_row_jacobian_writer.cc
    compressed_row_sparse_matrix.cc
    conditioned_cost_function.cc
    context.cc
    context_impl.cc
    coordinate_descent_minimizer.cc
    corrector.cc
    cost_function.cc
    covariance.cc
    covariance_impl.cc
    cuda_block_sparse_crs_view.cc
    cuda_partitioned_block_sparse_crs_view.cc
    cuda_block_structure.cc
    cuda_sparse_matrix.cc
    cuda_vector.cc
    dense_cholesky.cc
    dense_normal_cholesky_solver.cc
    dense_qr.cc
    dense_qr_solver.cc
    dense_sparse_matrix.cc
    detect_structure.cc
    dogleg_strategy.cc
    dynamic_compressed_row_jacobian_writer.cc
    dynamic_compressed_row_sparse_matrix.cc
    dynamic_sparse_normal_cholesky_solver.cc
    eigensparse.cc
    evaluation_callback.cc
    evaluator.cc
    fake_bundle_adjustment_jacobian.cc
    file.cc
    first_order_function.cc
    float_suitesparse.cc
    function_sample.cc
    gradient_checking_cost_function.cc
    gradient_problem_solver.cc
    implicit_schur_complement.cc
    inner_product_computer.cc
    is_close.cc
    iteration_callback.cc
    iterative_refiner.cc
    iterative_schur_complement_solver.cc
    levenberg_marquardt_strategy.cc
    line_search.cc
    line_search_direction.cc
    line_search_minimizer.cc
    line_search_preprocessor.cc
    linear_least_squares_problems.cc
    linear_operator.cc
    linear_solver.cc
    low_rank_inverse_hessian.cc
    minimizer.cc
    parallel_invoke.cc
    parallel_utils.cc
    parallel_vector_ops.cc
    parameter_block_ordering.cc
    partitioned_matrix_view.cc
    polynomial.cc
    power_series_expansion_preconditioner.cc
    preconditioner.cc
    preprocessor.cc
    problem_impl.cc
    program.cc
    reorder_program.cc
    residual_block.cc
    residual_block_utils.cc
    schur_complement_solver.cc
    schur_eliminator.cc
    schur_jacobi_preconditioner.cc
    schur_templates.cc
    scratch_evaluate_preparer.cc
    single_linkage_clustering.cc
    solver_utils.cc
    sparse_cholesky.cc
    sparse_matrix.cc
    sparse_normal_cholesky_solver.cc
    stringprintf.cc
    subset_preconditioner.cc
    suitesparse.cc
    thread_pool.cc
    thread_token_provider.cc
    triplet_sparse_matrix.cc
    trust_region_minimizer.cc
    trust_region_preprocessor.cc
    trust_region_step_evaluator.cc
    trust_region_strategy.cc
    visibility.cc
    visibility_based_preconditioner.cc
    wall_time.cc
)

set(CERES_LIBRARY_SOURCE
    ${CERES_INTERNAL_SRC}
    ${CERES_EXPORTED_SRCS}
    ${CERES_INTERNAL_HDRS}
    ${CERES_PUBLIC_HDRS}
    ${CERES_PUBLIC_INTERNAL_HDRS}
    ${CERES_INTERNAL_SCHUR_FILES}
        )

# Primarily for Android, but optionally for others, compile the minimal
# glog implementation into Ceres.
if (MINIGLOG)
  list(APPEND CERES_LIBRARY_SOURCE miniglog/glog/logging.cc)
endif (MINIGLOG)

# Ceres C++ compiler flags can be too strict for an external library code
# which we do not maintain.
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-Wno-missing-declarations"
                        CHECK_CXX_FLAG_Wno_missing_declarations)
if (CHECK_CXX_FLAG_Wno_missing_declarations)
  set_property(SOURCE gmock_gtest_all.cc
               APPEND_STRING PROPERTY COMPILE_FLAGS "-Wno-missing-declarations")
endif()

add_library(ceres STATIC ${CERES_LIBRARY_SOURCE})

# Ensure the minimum required C++ language version is fulfilled as our
# requirement by downstream clients. Consumers can choose the same or a newer
# language standard revision.
target_compile_features(ceres PUBLIC cxx_std_17)

set_target_properties(ceres PROPERTIES
  VERSION ${CERES_VERSION}
  SOVERSION 4)

target_link_libraries(ceres
  PUBLIC ${CERES_LIBRARY_PUBLIC_DEPENDENCIES}
  PRIVATE ${CERES_LIBRARY_PRIVATE_DEPENDENCIES})

# Add the Ceres headers to its target.
#
# Force the location containing the configured config.h to the front of the
# include_directories list (by default it is appended to the back) to ensure
# that if the user has an installed version of Ceres in the same location as one
# of the dependencies (e.g. /usr/local) that we find the config.h we just
# configured, not the (older) installed config.h.
target_include_directories(ceres
  BEFORE PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
  PRIVATE ${Ceres_SOURCE_DIR}/internal
          ${CERES_LIBRARY_PRIVATE_DEPENDENCIES_INCLUDE_DIRS}
  PUBLIC $<BUILD_INTERFACE:${Ceres_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

# Gather the list of public & private include locations for all enabled optional
# dependencies to be added to the Ceres target.
set(CERES_LIBRARY_PRIVATE_DEPENDENCIES_INCLUDE_DIRS "")
set(CERES_LIBRARY_PUBLIC_DEPENDENCIES_INCLUDE_DIRS "")

IF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
  SET_TARGET_PROPERTIES(ceres PROPERTIES FOLDER external)
ENDIF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)

SET_TARGET_PROPERTIES(ceres PROPERTIES
                ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_BINARY_DIR}/../../
                ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_BINARY_DIR}/../../)
SET_TARGET_PROPERTIES(ceres PROPERTIES 
                LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_BINARY_DIR}/../../
                LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_BINARY_DIR}/../../)
SET_TARGET_PROPERTIES(ceres PROPERTIES
                RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_BINARY_DIR}/../../
                RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_BINARY_DIR}/../../)

# Generate an export header for annotating symbols visibility
include(GenerateExportHeader)
generate_export_header(ceres EXPORT_FILE_NAME
  ${CMAKE_CURRENT_BINARY_DIR}/../../include/ceres/internal/export.h)
