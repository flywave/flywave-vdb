# Ceres Solver - A fast non-linear least squares minimizer
# Copyright 2015 Google Inc. All rights reserved.
# http://ceres-solver.org/
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
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

set(CERES_INTERNAL_SRC
    accelerate_sparse.cc
    array_utils.cc
    blas.cc
    block_evaluate_preparer.cc
    block_jacobi_preconditioner.cc
    block_jacobian_writer.cc
    block_random_access_dense_matrix.cc
    block_random_access_diagonal_matrix.cc
    block_random_access_matrix.cc
    block_random_access_sparse_matrix.cc
    block_sparse_matrix.cc
    block_structure.cc
    c_api.cc
    canonical_views_clustering.cc
    cgnr_solver.cc
    callbacks.cc
    compressed_col_sparse_matrix_utils.cc
    compressed_row_jacobian_writer.cc
    compressed_row_sparse_matrix.cc
    conditioned_cost_function.cc
    conjugate_gradients_solver.cc
    context.cc
    context_impl.cc
    coordinate_descent_minimizer.cc
    corrector.cc
    covariance.cc
    covariance_impl.cc
    cxsparse.cc
    dense_normal_cholesky_solver.cc
    dense_qr_solver.cc
    dense_sparse_matrix.cc
    detect_structure.cc
    dogleg_strategy.cc
    dynamic_compressed_row_jacobian_writer.cc
    dynamic_compressed_row_sparse_matrix.cc
    dynamic_sparse_normal_cholesky_solver.cc
    evaluator.cc
    eigensparse.cc
    file.cc
    function_sample.cc
    gradient_checker.cc
    gradient_checking_cost_function.cc
    gradient_problem.cc
    gradient_problem_solver.cc
    implicit_schur_complement.cc
    inner_product_computer.cc
    is_close.cc
    iterative_schur_complement_solver.cc
    levenberg_marquardt_strategy.cc
    lapack.cc
    line_search.cc
    line_search_direction.cc
    line_search_minimizer.cc
    line_search_preprocessor.cc
    linear_least_squares_problems.cc
    linear_operator.cc
    linear_solver.cc
    local_parameterization.cc
    loss_function.cc
    low_rank_inverse_hessian.cc
    minimizer.cc
    normal_prior.cc
    parallel_for_cxx.cc
    parallel_for_tbb.cc
    parallel_utils.cc
    parameter_block_ordering.cc
    partitioned_matrix_view.cc
    polynomial.cc
    preconditioner.cc
    preprocessor.cc
    problem.cc
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
    solver.cc
    solver_utils.cc
    sparse_matrix.cc
    sparse_cholesky.cc
    sparse_normal_cholesky_solver.cc
    subset_preconditioner.cc
    split.cc
    stringprintf.cc
    suitesparse.cc
    thread_pool.cc
    thread_token_provider.cc
    triplet_sparse_matrix.cc
    trust_region_preprocessor.cc
    trust_region_minimizer.cc
    trust_region_step_evaluator.cc
    trust_region_strategy.cc
    types.cc
    visibility.cc
    visibility_based_preconditioner.cc
    wall_time.cc
)

# Also depend on the header files so that they appear in IDEs.
file(GLOB CERES_INTERNAL_HDRS *.h)
if (MINIGLOG)
  file(GLOB MINIGLOG_HDRS miniglog/glog/*.h)
  list(APPEND CERES_INTERNAL_HDRS ${MINIGLOG_HDRS})
endif()

# Depend also on public headers so they appear in IDEs.
file(GLOB CERES_PUBLIC_HDRS ${Ceres_SOURCE_DIR}/include/ceres/*.h)
file(GLOB CERES_PUBLIC_INTERNAL_HDRS ${Ceres_SOURCE_DIR}/include/ceres/internal/*.h)

# Include the specialized schur solvers.
if (SCHUR_SPECIALIZATIONS)
  file(GLOB CERES_INTERNAL_SCHUR_FILES generated/*.cc)
else (SCHUR_SPECIALIZATIONS)
  # Only the fully dynamic solver. The build is much faster this way.
  file(GLOB CERES_INTERNAL_SCHUR_FILES generated/*_d_d_d.cc)
endif (SCHUR_SPECIALIZATIONS)

# Build the list of dependencies for Ceres based on the current configuration.
if (NOT MINIGLOG AND GLOG_FOUND)
  list(APPEND CERES_LIBRARY_PUBLIC_DEPENDENCIES ${GLOG_LIBRARIES})
  if (GFLAGS_FOUND)
    # If glog & gflags are both found, we assume that glog was built with
    # gflags, as it is awkward to perform a try_compile() to verify this
    # when gflags is an imported target (as it is in newer versions).
    # As glog #includes gflags/gflags.h in glog/logging.h if compiled with
    # gflags, it is thus a public dependency for Ceres in this case.
    list(APPEND CERES_LIBRARY_PUBLIC_DEPENDENCIES ${GFLAGS_LIBRARIES})
  endif()
endif (NOT MINIGLOG AND GLOG_FOUND)

# Define version information for use in Solver::FullReport.
add_definitions(-DCERES_CXSPARSE_VERSION="3.1.1")
list(APPEND CERES_LIBRARY_PRIVATE_DEPENDENCIES ${CXSPARSE_LIBRARIES})

if (LAPACK_FOUND)
  list(APPEND CERES_LIBRARY_PRIVATE_DEPENDENCIES ${LAPACK_LIBRARIES})
endif ()

set(CERES_LIBRARY_SOURCE
    ${CERES_INTERNAL_SRC}
    ${CERES_INTERNAL_HDRS}
    ${CERES_PUBLIC_HDRS}
    ${CERES_PUBLIC_INTERNAL_HDRS}
    ${CERES_INTERNAL_SCHUR_FILES})

if (MINIGLOG)
  list(APPEND CERES_LIBRARY_SOURCE miniglog/glog/logging.cc)
endif (MINIGLOG)

add_library(ceres STATIC ${CERES_LIBRARY_SOURCE})
set_target_properties(ceres PROPERTIES
  VERSION ${CERES_VERSION}
  SOVERSION ${CERES_VERSION_MAJOR})

IF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
  SET_TARGET_PROPERTIES(ceres PROPERTIES FOLDER external)
ENDIF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)

include(AppendTargetProperty)
# Always build position-independent code (PIC), even when building Ceres as a
# static library so that shared libraries can link against it, not just
# executables (PIC does not apply on Windows).
if (NOT WIN32 AND NOT BUILD_SHARED_LIBS)
  # Use the explicit POSITION_INDEPENDENT_CODE target property on CMake versions
  # that support it (>= 2.8.9).  Otherwise, manually add the -fPIC flag as an
  # additional compile definitions for the target.
  if (CMAKE_VERSION VERSION_LESS "2.8.9")
    append_target_property(ceres COMPILE_FLAGS "-fPIC")
  else()
    # Use set_target_properties() not append_target_property() here as
    # POSITION_INDEPENDENT_CODE is a binary ON/OFF switch.
    set_target_properties(ceres PROPERTIES POSITION_INDEPENDENT_CODE ON)
  endif()
endif()

if (CXX11)
  include(AddCeresCXX11RequirementsToTarget)
  add_ceres_cxx11_requirements_to_target(ceres)
endif()

if (BUILD_SHARED_LIBS)
  # When building a shared library, mark all external libraries as
  # PRIVATE so they don't show up as a dependency.
  target_link_libraries(ceres
        LINK_PUBLIC ${CERES_LIBRARY_PUBLIC_DEPENDENCIES}
        LINK_PRIVATE ${CERES_LIBRARY_PRIVATE_DEPENDENCIES})
else (BUILD_SHARED_LIBS)
  # When building a static library, all external libraries are
  # PUBLIC(default) since the user needs to link to them.
  # They will be listed in CeresTargets.cmake.
  set(CERES_LIBRARY_DEPENDENCIES
        ${CERES_LIBRARY_PUBLIC_DEPENDENCIES}
        ${CERES_LIBRARY_PRIVATE_DEPENDENCIES})
  target_link_libraries(ceres ${CERES_LIBRARY_DEPENDENCIES})
endif (BUILD_SHARED_LIBS)
