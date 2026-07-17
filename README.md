# flywave-vdb

**Go package that wraps a large C++ voxel / 3D rendering stack** — OpenVDB
generation & manipulation, tungsten PBR rendering, embree ray tracing, volume
ray marching, and Gaussian splatting support — through a small C API.

## Architecture

```
┌─────────────────────────────────────┐
│         Go API  (vdb package)       │  ← .go files, one per C header
├─────────────────────────────────────┤
│   C API  headers in lib/ & src/     │  ← grid_api.h, voxelizer_api.h,
│                                     │     renderer_api.h, gaussplat_api.h
├─────────────────────────────────────┤
│   C++ implementation in src/        │  ← ~70 .cc/.hh files, bridge to
│                                     │     vendored libraries
├─────────────────────────────────────┤
│   Vendored C/C++ libraries          │  ← external/ (embree, openvdb, etc.)
│   in external/                      │
└─────────────────────────────────────┘
```

Each `.go` file in the root directory CGo-wraps one C header from `src/`. The
C/C++ layer is compiled into a single static archive `libvdb.a`, which the Go
test binary links against.

## Build

> **Prerequisites**: CMake ≥ 3.28, GCC ≥ 13, Go ≥ 1.21

```sh
# 1. Configure & build the C++ stack (~30 static libraries)
cmake -S . -B build                 # optional: -DEMBREE_ISPC_SUPPORT=OFF
cmake --build build                 # Release by default

# 2. Install .a + headers into ./lib/
cmake --install build

# 3. Build/Test the Go layer
go build ./...
go test ./...
```

The Go side will **not** compile until `./lib/` is populated. If you see
`grid_api.h` not found or undefined C symbols, re-run steps 1–2.

## Vendored dependencies

The C++ stack ships ~40 vendored libraries under `external/`. Major ones:

| Library | Version | Purpose |
|---|---|---|
| OpenVDB | **13.0.0** | Sparse volumetric grid engine |
| embree | **4.3.3** | Ray tracing kernels (BVH) |
| tungsten | (vendored) | PBR path tracer |
| ceres-solver | **2.2.0** | Non-linear least squares |
| OIIO | **3.0.13** | Image I/O |
| OpenEXR | **3.4.2** | EXR image I/O |
| Imath | **3.2.2** | Half-float math |
| Boost | **1.88.0** | C++ support libs |
| TBB | **2021.1.1** | Parallel algorithms |
| eigen | **3.4.1** | Linear algebra |
| suitesparse | 5.7.1 | Sparse solvers |
| fmt | **12.0.0** | String formatting |
| openjph | (vendored) | JPEG-XL HT codec |

All vendored libraries are built as **static archives** and linked into the Go
test binary via the CGo LDFLAGS in `grid.go`.

## Features

### VDB Grid Operations
- Create grids from points or mesh (`NewFloatGridFromPoints`, `NewFloatGridFromMesh`)
- Read/write `.vdb` files
- Boolean operations (union/difference/intersection)
- Smooth, offset, blend, clip
- SDF level set reconstruction
- Active voxel traversal (`VisitOn`, `VisitAll`, `VisitOff`)
- Grid transformation (index ↔ world, voxel size, frustum)

### Voxelization & Meshing
- `VoxelMeshBuilder`: build mesh from triangles, sample to voxel pixels
- `MeshModel`: construct primitives (box, sphere, etc.)
- Extract triangle meshes from grids with iso-surface
- Texture mesh creation from triangle data

### Tiled Indexing
- Quaternary tile index (Quadtree on sphere)
- Level/row/col ↔ bitlist ↔ path conversion
- Parent/child navigation, relative paths
- Lon/lat ↔ Mercator projection

### Rendering
- PBR renderer via tungsten (`Renderer`)
- Multi-threaded scene rendering
- Frame buffer output (`GetFrameBuffer`, `GetFrameBufferImage`)
- Denoiser support
- Checkpoint & timeout control

### Volume Ray Marching (new in v0.1.0)
- Transfer function: 1024-entry color LUT with linear interpolation
- `FloatGrid.RayMarch`: single ray, trilinear sampling, front-to-back compositing
- Batch ray marching (`vdb_grid_ray_march_many`)
- Early termination on opacity saturation

### Multi-View Camera (new in v0.1.0)
- `CameraOrbit`: spherical coordinate control
- `CameraOrbitPath`: automatic spiral camera generation for multi-view datasets

### Gaussian Splatting Support (new in v0.1.0)
- Transfer function API for scalar → color/opacity mapping
- Volume ray marching for training data generation
- Camera orbit paths for multi-view capture
- (Training pipeline integrated with external 3DGS frameworks)

## Project layout

```
.
├── *.go                  # Go CGo wrappers (one per C API)
├── *_test.go             # White-box tests (package vdb)
├── grid.go               # CGo LDFLAGS (source of truth for link line)
├── go.mod                # Go module (requires Go ≥ 1.21)
│
├── src/                  # C++ implementation + C API headers
│   ├── *.hh / *.cc       # ~70 files
│   ├── grid_api.h        # VDB grid C API
│   ├── voxelizer_api.h   # Voxelization C API
│   ├── renderer_api.h    # Rendering C API
│   ├── gaussplat_api.h   # Gaussian splatting C API (transfer function,
│   │                      # ray march, multi-view camera)
│   └── CMakeLists.txt    # Builds libvdb.a
│
├── coord/                # Pure-Go coordinate sub-package
│   ├── coord.go / box.go
│   └── coord_test.go
│
├── external/             # Vendored C/C++ libraries (~40)
│   ├── openvdb/          # OpenVDB 13.0.0
│   ├── embree/           # embree 4.3.3
│   ├── booser/           # Boost 1.88.0
│   ├── tbb/              # oneTBB 2021.1.1
│   ├── …other deps…
│
├── lib/                  # Built .a archives + .h headers (gitignored)
│
├── build/                # CMake build tree (gitignored)
│
└── CMakeLists.txt        # Top-level CMake orchestration
```

## Tests

- **Go tests** are white-box (`package vdb`) and live next to the source.  
  Because CGo LDFLAGS are package-wide, every test links the full C++ stack.
- **C++ test** target `test` (from `src/test.cc`) is built by CMake.
- Run a single Go test with:
  ```sh
  go test -run TestMeshBuilder ./...
  ```
- Current test count: **89 tests** across `vdb` + `coord` packages.

### Test areas

| Package | Tests | Covers |
|---|---|---|
| `coord` | 15 | Vector/math/box basics |
| `vdb` | 74 | Grids, meshes, tiles, transforms, rendering, transfer function, ray march, cache |

## License

See LICENSE file.
