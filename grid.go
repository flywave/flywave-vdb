package vdb

// #include <stdlib.h>
// #include "vdb_api.h"
// #cgo CFLAGS: -I ./lib
// #cgo CXXFLAGS: -I ./lib
// #cgo linux LDFLAGS:  -L ./lib -L /usr/lib/x86_64-linux-gnu -Wl,--start-group  -lstdc++ -lm -pthread -ldl -lamd -lamd_l -lbin_packer -lblosc -lboost_chrono_internal -lboost_date_time_internal -lboost_filesystem_internal -lboost_iostreams_internal -lboost_log_internal -lboost_program_options_internal -lboost_regex_internal -lboost_serialization_internal -lboost_system_internal -lboost_test_internal -lboost_thread_internal -lboost_timer_internal -lcamd -lcamd_l -lccolamd -lccolamd_l -lceres -lcholmod -lcholmod_l -lcolamd -lcolamd_l -lembree -lexpat -lfmt -lfontconfig -lfreetype -lGKlib -liconv -licudata -licudt -licui18n -licutools -licuuc -lIlmImf -ljasper -ljpeg -llerc -llexers -llz4 -llzma -lmath -lmetis -lOpenImageIO -lOpenImageIO_Util -lopenvdb -lpng -lsimd -lskylight -lsnappy -lspqr -lSuiteSparse_config -lsys -ltasking -ltbb -ltbbmalloc -ltbbmalloc_proxy -ltiff -lvdb -lwebp -lxatlas -lxml2 -lzlib -lzstd -ltungsten -ltungsten_thirdparty -Wl,--end-group
// #cgo windows LDFLAGS: -L ./lib　-lamd -lamd_l -lbin_packer -lblosc -lboost_chrono_internal -lboost_date_time_internal -lboost_filesystem_internal -lboost_iostreams_internal -lboost_log_internal -lboost_program_options_internal -lboost_regex_internal -lboost_serialization_internal -lboost_system_internal -lboost_test_internal -lboost_thread_internal -lboost_timer_internal -lcamd -lcamd_l -lccolamd -lccolamd_l -lceres -lcholmod -lcholmod_l -lcolamd -lcolamd_l -lembree -lexpat -lfmt -lfontconfig -lfreetype -lGKlib -liconv -licudata -licudt -licui18n -licutools -licuuc -lIlmImf -ljasper -ljpeg -llerc -llexers -llz4 -llzma -lmath -lmetis -lOpenImageIO -lOpenImageIO_Util -lopenvdb -lpng -lsimd -lskylight -lsnappy -lspqr -lSuiteSparse_config -lsys -ltasking -ltbb -ltbbmalloc -ltbbmalloc_proxy -ltiff -lvdb -lwebp -lxatlas -lxml2 -lzlib -lzstd -ltungsten -ltungsten_thirdparty
// #cgo darwin LDFLAGS: -L　./lib -lamd -lamd_l -lbin_packer -lblosc -lboost_chrono_internal -lboost_date_time_internal -lboost_filesystem_internal -lboost_iostreams_internal -lboost_log_internal -lboost_program_options_internal -lboost_regex_internal -lboost_serialization_internal -lboost_system_internal -lboost_test_internal -lboost_thread_internal -lboost_timer_internal -lcamd -lcamd_l -lccolamd -lccolamd_l -lceres -lcholmod -lcholmod_l -lcolamd -lcolamd_l -lembree -lexpat -lfmt -lfontconfig -lfreetype -lGKlib -liconv -licudata -licudt -licui18n -licutools -licuuc -lIlmImf -ljasper -ljpeg -llerc -llexers -llz4 -llzma -lmath -lmetis -lOpenImageIO -lOpenImageIO_Util -lopenvdb -lpng -lsimd -lskylight -lsnappy -lspqr -lSuiteSparse_config -lsys -ltasking -ltbb -ltbbmalloc -ltbbmalloc_proxy -ltiff -lvdb -lwebp -lxatlas -lxml2 -lzlib -lzstd -ltungsten -ltungsten_thirdparty
import "C"
import (
	"errors"
	"reflect"
	"unsafe"
)

type SmoothType int32

const (
	SMOOTH_GAUSSIAN       = SmoothType(0)
	SMOOTH_LAPLACIAN      = SmoothType(1)
	SMOOTH_MEAN           = SmoothType(2)
	SMOOTH_MEDIAN         = SmoothType(3)
	SMOOTH_MEAN_CURVATURE = SmoothType(4)
)

type Grid struct {
	m *C.struct__vdb_grid_t
}

func NewGrid() *Grid {
	return &Grid{
		m: C.vdb_create(),
	}
}

func NewGridFromPoints(points []float64, radius []float64, voxelSize float64, bandwidth float64) *Grid {
	if voxelSize < 0.01 {
		voxelSize = 0.01
	}

	if bandwidth < 1 {
		bandwidth = 1
	}

	var g = &Grid{
		m: C.vdb_create(),
	}
	C.vdb_from_points(g.m, (*C.double)(unsafe.Pointer(&points[0])), C.int(len(points)), (*C.double)(unsafe.Pointer(&radius[0])), C.int(len(radius)), C.double(voxelSize), C.double(bandwidth))
	return g
}

func NewGridFromMesh(points []float32, faces []int32, voxelSize float64, bandwidth float64) *Grid {
	if voxelSize < 0.01 {
		voxelSize = 0.01
	}

	if bandwidth < 1 {
		bandwidth = 1
	}

	var g = &Grid{
		m: C.vdb_create(),
	}
	C.vdb_from_mesh(g.m, (*C.float)(unsafe.Pointer(&points[0])), C.int(len(points)), (*C.int)(unsafe.Pointer(&faces[0])), C.int(len(faces)), C.double(voxelSize), C.double(bandwidth))
	return g
}

func (m *Grid) Free() {
	C.vdb_free(m.m)
	m.m = nil
}

func (m *Grid) Clone() *Grid {
	return &Grid{
		m: C.vdb_duplicate(m.m),
	}
}

func (m *Grid) Read(file string) error {
	if m == nil || m.m == nil {
		return errors.New("Grid error ")
	}

	fname := C.CString(file)
	defer C.free(unsafe.Pointer(fname))
	C.vdb_read(m.m, fname)

	return nil
}

func (m *Grid) Write(file string) error {
	if m == nil || m.m == nil {
		return errors.New("Grid error ")
	}

	fname := C.CString(file)
	defer C.free(unsafe.Pointer(fname))
	C.vdb_write(m.m, fname)

	return nil
}

func (m *Grid) ToMesh() ([]float32, []int32, error) {
	if m == nil || m.m == nil {
		return nil, nil, errors.New("Grid error ")
	}

	C.vdb_to_mesh(m.m)

	var pointsLen C.int
	cpoints := C.vdb_vertex_buffer(m.m, &pointsLen)

	var cpointsSlice []C.float
	cpointsHeader := (*reflect.SliceHeader)((unsafe.Pointer(&cpointsSlice)))
	cpointsHeader.Cap = int(pointsLen)
	cpointsHeader.Len = int(pointsLen)
	cpointsHeader.Data = uintptr(unsafe.Pointer(cpoints))

	points := make([]float32, int(pointsLen))
	for i := 0; i < int(pointsLen); i++ {
		points[i] = float32(cpointsSlice[i])
	}

	C.free(unsafe.Pointer(cpoints))

	var facesLen C.int
	cfaces := C.vdb_face_buffer(m.m, &facesLen)

	var cfacesSlice []C.int
	cfacesHeader := (*reflect.SliceHeader)((unsafe.Pointer(&cfacesSlice)))
	cfacesHeader.Cap = int(facesLen)
	cfacesHeader.Len = int(facesLen)
	cfacesHeader.Data = uintptr(unsafe.Pointer(cfaces))

	faces := make([]int32, int(facesLen))
	for i := 0; i < int(facesLen); i++ {
		faces[i] = int32(cfacesSlice[i])
	}
	C.free(unsafe.Pointer(cfaces))

	return points, faces, nil
}

func (m *Grid) ToMeshSettings(isovalue float64, adaptivity float64) ([]float32, []int32, error) {
	if m == nil || m.m == nil {
		return nil, nil, errors.New("Grid error ")
	}
	C.vdb_to_mesh_settings(m.m, C.double(isovalue), C.double(adaptivity))

	var pointsLen C.int
	cpoints := C.vdb_vertex_buffer(m.m, &pointsLen)

	var cpointsSlice []C.float
	cpointsHeader := (*reflect.SliceHeader)((unsafe.Pointer(&cpointsSlice)))
	cpointsHeader.Cap = int(pointsLen)
	cpointsHeader.Len = int(pointsLen)
	cpointsHeader.Data = uintptr(unsafe.Pointer(cpoints))

	points := make([]float32, int(pointsLen))
	for i := 0; i < int(pointsLen); i++ {
		points[i] = float32(cpointsSlice[i])
	}
	C.free(unsafe.Pointer(cpoints))

	var facesLen C.int
	cfaces := C.vdb_face_buffer(m.m, &facesLen)

	var cfacesSlice []C.int
	cfacesHeader := (*reflect.SliceHeader)((unsafe.Pointer(&cfacesSlice)))
	cfacesHeader.Cap = int(facesLen)
	cfacesHeader.Len = int(facesLen)
	cfacesHeader.Data = uintptr(unsafe.Pointer(cfaces))

	faces := make([]int32, int(facesLen))
	for i := 0; i < int(facesLen); i++ {
		faces[i] = int32(cfacesSlice[i])
	}
	C.free(unsafe.Pointer(cfaces))

	return points, faces, nil
}

func (m *Grid) Transform(matrix []float64) (bool, error) {
	if m == nil || m.m == nil {
		return false, errors.New("Grid error ")
	}
	ret := C.vdb_transform(m.m, (*C.double)(unsafe.Pointer(&matrix[0])), C.int(len(matrix)))
	return bool(ret), nil
}

func (m *Grid) BooleanUnion(csg *Grid) error {
	if m == nil || m.m == nil {
		return errors.New("Union error ")
	}
	C.vdb_union(m.m, csg.m)
	return nil
}

func (m *Grid) BooleanDifference(csg *Grid) error {
	if m == nil || m.m == nil {
		return errors.New("Difference error ")
	}
	C.vdb_difference(m.m, csg.m)
	return nil
}

func (m *Grid) BooleanIntersection(csg *Grid) error {
	if m == nil || m.m == nil {
		return errors.New("Intersection error ")
	}
	C.vdb_intersection(m.m, csg.m)
	return nil
}

func (m *Grid) Offset(amount float64) error {
	if m == nil || m.m == nil {
		return errors.New("Offset error ")
	}
	C.vdb_offset(m.m, C.double(amount))
	return nil
}

func (m *Grid) OffsetMask(amount float64, mask *Grid, min float64, max float64, invert bool) error {
	if m == nil || m.m == nil {
		return errors.New("OffsetMask error ")
	}
	C.vdb_offset_mask(m.m, C.double(amount), mask.m, C.double(min), C.double(max), C.bool(invert))
	return nil
}

func (m *Grid) Smooth(type_ SmoothType, iterations int32, width int32) error {
	if m == nil || m.m == nil {
		return errors.New("Smooth error ")
	}
	if width < 1 {
		width = 1
	}
	if iterations < 1 {
		iterations = 1
	}
	C.vdb_smooth(m.m, C.int(type_), C.int(iterations), C.int(width))
	return nil
}

func (m *Grid) SmoothMask(type_ SmoothType, iterations int32, width int32, mask *Grid, min float64, max float64, invert bool) error {
	if m == nil || m.m == nil {
		return errors.New("SmoothMask error ")
	}
	if width < 1 {
		width = 1
	}
	if iterations < 1 {
		iterations = 1
	}
	C.vdb_smooth_mask(m.m, C.int(type_), C.int(iterations), C.int(width), mask.m, C.double(min), C.double(max), C.bool(invert))
	return nil
}

func (m *Grid) Blend(grid *Grid, position float64, end float64) error {
	if m == nil || m.m == nil {
		return errors.New("Blend error ")
	}
	if position < 0 {
		position = 0
	}
	if position > 1 {
		position = 1
	}
	if end < 1 {
		end = 1
	}
	position = 1 - position
	C.vdb_blend(m.m, grid.m, C.double(position), C.double(end))
	return nil
}

func (m *Grid) BlendMask(grid *Grid, position float64, end float64, mask *Grid, min float64, max float64, invert bool) error {
	if m == nil || m.m == nil {
		return errors.New("BlendMask error ")
	}
	if position < 0 {
		position = 0
	}
	if position > 1 {
		position = 1
	}
	if end < 1 {
		end = 1
	}
	position = 1 - position
	C.vdb_blend_mask(m.m, grid.m, C.double(position), C.double(end), mask.m, C.double(min), C.double(max), C.bool(invert))
	return nil
}

func (m *Grid) ClosestPoint(points []float32) ([]float32, error) {
	if m == nil || m.m == nil {
		return nil, errors.New("ClosestPoint error ")
	}

	var pointsLen C.int
	cpoints := C.vdb_closest_point(m.m, (*C.float)(unsafe.Pointer(&points[0])), C.int(len(points)), &pointsLen)

	var cpointsSlice []C.float
	cpointsHeader := (*reflect.SliceHeader)((unsafe.Pointer(&cpointsSlice)))
	cpointsHeader.Cap = int(pointsLen)
	cpointsHeader.Len = int(pointsLen)
	cpointsHeader.Data = uintptr(unsafe.Pointer(cpoints))

	closest_points := make([]float32, int(pointsLen))
	for i := 0; i < int(pointsLen); i++ {
		closest_points[i] = float32(cpointsSlice[i])
	}
	C.free(unsafe.Pointer(cpoints))

	return closest_points, nil
}

func (m *Grid) Rebuild(iso float64, exWidth float64, inWidth float64) error {
	if m == nil || m.m == nil {
		return errors.New("Rebuild error ")
	}
	C.vdb_rebuild(m.m, C.float(iso), C.float(exWidth), C.float(inWidth))
	return nil
}

func (m *Grid) Dense() ([]float32, [3]int32, error) {
	if m == nil || m.m == nil {
		return nil, [3]int32{0, 0, 0}, errors.New("Rebuild error ")
	}

	var width C.int
	var height C.int
	var depth C.int
	cdense := C.vdb_dense(m.m, &width, &height, &depth)
	allsize := int(width * height * depth)

	var cdenseSlice []C.float
	cdenseHeader := (*reflect.SliceHeader)((unsafe.Pointer(&cdenseSlice)))
	cdenseHeader.Cap = int(allsize)
	cdenseHeader.Len = int(allsize)
	cdenseHeader.Data = uintptr(unsafe.Pointer(cdense))
	dense := make([]float32, allsize)
	for i := 0; i < allsize; i++ {
		dense[i] = float32(cdenseSlice[i])
	}
	C.free(unsafe.Pointer(cdense))
	return dense, [3]int32{int32(width), int32(height), int32(depth)}, nil
}
