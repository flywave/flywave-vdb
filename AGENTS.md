# AGENTS.md

Go package that exposes a large C++ voxel / 3D-rendering stack (OpenVDB, embree,
ceres, OpenEXR, OIIO, suitesparse, tungsten, ...) through a small C API.

## Build order matters (the one thing most likely to trip you up)

The Go side will **not** compile or link until the C++ side is built and
*installed* into `./lib/`. `lib/` is gitignored and is produced only by the
CMake install step. CGo directives across the package read
`-I ./lib` (for headers like `grid_api.h`) and `-L ./lib -lvdb -lopenvdb ...`
(dozens of static libs, see `grid.go`).

Correct sequence:

```sh
cmake -S . -B build                 # optional: -DFLYWAVE_USE_SSE2=ON ...
cmake --build build                 # Release by default; builds ~30 static libs
cmake --install build               # copies *.a + src/*.h into ./lib/
go build ./...                      # only now will this link
```

If `go build` fails with missing `grid_api.h` or undefined C symbols, the cause
is almost always a missing/stale `./lib/` — rebuild + reinstall C++, do not patch
the `.go` files.

## Layout

- Repo root = Go package `vdb` (module `github.com/flywave/flywave-vdb`).
  Each `.go` file here CGo-wraps one C header from `src/`.
- `coord/` = sub-package, also CGo.
- `src/` = C++ implementation + the C API headers (`grid_api.h`,
  `voxelizer_api.h`, `renderer_api.h`, `tile_index_api.h`). Built by CMake into
  `libvdb.a`. Entry points for new C API surface go here.
- `external/` = vendored third-party C/C++ libraries. **Do not edit**; treat as
  read-only. Listed individually in the root `CMakeLists.txt`.
- `data/voxel-medium/` = sample data used by tests.
- `cmake/FindSSE.cmake` = SSE detection helper.

## Tests

- Go tests are white-box (`package vdb`) next to the source (`*_test.go`).
- Because CGo LDFLAGS are package-wide, **every** test in package `vdb` links
  the full C++ stack — even the pure-Go ones (`vdb_test.go`, `tile_test.go`).
  `go test ./...` therefore requires a populated `./lib/`.
  Run a single test with: `go test -run TestMeshBuilder ./...`
- There is also a C++ test executable target `test` (from `src/test.cc`),
  built by CMake; run it from the build dir after `cmake --build`.

## Toolchain notes

- `go.mod` declares `go 1.12`. C++ requires C++17 (`-std=c++17`).
- Unix builds use `-fPIC` and `-march=native`.
- Output is static libraries only (`*.a`); the install step matches
  `PATTERN "*.a"` and copies public headers `*.h` (not `.hh`).
- No CI, lint, formatter, or typecheck config exists. No top-level Makefile —
  use `cmake` + `go` directly.

## Editing gotchas

- The `// #cgo ... LDFLAGS:` lines in `grid.go` (and mirrors in other files)
  are the source of truth for the link line on each platform; keep them in sync
  when changing C++ dependencies.
- The `windows` and `darwin` LDFLAGS lines contain full-width spaces (`　`)
  after `-L`; preserve them verbatim or rewrite all three platforms together.
- `lerc` and `snappy` blocks are duplicated in the root `CMakeLists.txt` —
  harmless, but don't "fix" unless you understand why.
