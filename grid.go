package vdb

// #include <stdlib.h>
// #include "grid_api.h"
// #cgo CFLAGS: -I ./lib
// #cgo CXXFLAGS: -I ./lib
// #cgo linux LDFLAGS:  -L ./lib -L /usr/lib/x86_64-linux-gnu -Wl,--start-group  -lstdc++ -lm -pthread -ldl -lamd -lamd_l -lbin_packer -lblosc -lboost_chrono_internal -lboost_date_time_internal -lboost_filesystem_internal -lboost_iostreams_internal -lboost_log_internal -lboost_program_options_internal -lboost_regex_internal -lboost_serialization_internal -lboost_system_internal -lboost_test_internal -lboost_thread_internal -lboost_timer_internal -lcamd -lcamd_l -lccolamd -lccolamd_l -lceres -lcholmod -lcholmod_l -lcolamd -lcolamd_l -lembree -llexers -lsimd -lsys -lexpat -lfmt -lfontconfig -lfreetype -lGKlib -liconv -licudata -licudt -licui18n -licutools -licuuc -lIlmImf -ljasper -ljpeg -llerc -llexers -llz4 -llzma -lmetis -lOpenImageIO -lOpenImageIO_Util -lopenvdb -lpng -lsimd -lsnappy -lspqr -lSuiteSparse_config -lsys -ltbb -ltbbmalloc -ltbbmalloc_proxy -ltiff -lvdb -lwebp -lxatlas -lxml2 -lzlib -lzstd -ltungsten -ltungsten_thirdparty -ldenoiser -Wl,--end-group
// #cgo windows LDFLAGS: -L ./lib　-lamd -lamd_l -lbin_packer -lblosc -lboost_chrono_internal -lboost_date_time_internal -lboost_filesystem_internal -lboost_iostreams_internal -lboost_log_internal -lboost_program_options_internal -lboost_regex_internal -lboost_serialization_internal -lboost_system_internal -lboost_test_internal -lboost_thread_internal -lboost_timer_internal -lcamd -lcamd_l -lccolamd -lccolamd_l -lceres -lcholmod -lcholmod_l -lcolamd -lcolamd_l -lembree -llexers -lsimd -lsys -lexpat -lfmt -lfontconfig -lfreetype -lGKlib -liconv -licudata -licudt -licui18n -licutools -licuuc -lIlmImf -ljasper -ljpeg -llerc -llexers -llz4 -llzma -lmetis -lOpenImageIO -lOpenImageIO_Util -lopenvdb -lpng -lsimd -lsnappy -lspqr -lSuiteSparse_config -lsys -ltbb -ltbbmalloc -ltbbmalloc_proxy -ltiff -lvdb -lwebp -lxatlas -lxml2 -lzlib -lzstd -ltungsten -ltungsten_thirdparty -ldenoiser
// #cgo darwin LDFLAGS: -L　./lib -lamd -lamd_l -lbin_packer -lblosc -lboost_chrono_internal -lboost_date_time_internal -lboost_filesystem_internal -lboost_iostreams_internal -lboost_log_internal -lboost_program_options_internal -lboost_regex_internal -lboost_serialization_internal -lboost_system_internal -lboost_test_internal -lboost_thread_internal -lboost_timer_internal -lcamd -lcamd_l -lccolamd -lccolamd_l -lceres -lcholmod -lcholmod_l -lcolamd -lcolamd_l -lembree -llexers -lsimd -lsys -lexpat -lfmt -lfontconfig -lfreetype -lGKlib -liconv -licudata -licudt -licui18n -licutools -licuuc -lIlmImf -ljasper -ljpeg -llerc -llexers -llz4 -llzma -lmetis -lOpenImageIO -lOpenImageIO_Util -lopenvdb -lpng -lsimd -lsnappy -lspqr -lSuiteSparse_config -lsys -ltbb -ltbbmalloc -ltbbmalloc_proxy -ltiff -lvdb -lwebp -lxatlas -lxml2 -lzlib -lzstd -ltungsten -ltungsten_thirdparty -ldenoiser
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

type FloatGrid struct {
	m *C.struct__vdb_float_grid_t
}

func NewFloatGrid() *FloatGrid {
	return &FloatGrid{
		m: C.vdb_float_grid_create(),
	}
}

func NewFloatGridFromPoints(points []float64, radius []float64, voxelSize float64, bandwidth float64) *FloatGrid {
	if voxelSize < 0.01 {
		voxelSize = 0.01
	}

	if bandwidth < 1 {
		bandwidth = 1
	}

	var g = &FloatGrid{
		m: C.vdb_float_grid_create(),
	}
	C.vdb_float_grid_from_points(g.m, (*C.double)(unsafe.Pointer(&points[0])), C.int(len(points)), (*C.double)(unsafe.Pointer(&radius[0])), C.int(len(radius)), C.double(voxelSize), C.double(bandwidth))
	return g
}

func NewFloatGridFromMesh(points []float32, faces []int32, voxelSize float64, bandwidth float64) *FloatGrid {
	if voxelSize < 0.01 {
		voxelSize = 0.01
	}

	if bandwidth < 1 {
		bandwidth = 1
	}

	var g = &FloatGrid{
		m: C.vdb_float_grid_create(),
	}
	C.vdb_float_grid_from_mesh(g.m, (*C.float)(unsafe.Pointer(&points[0])), C.int(len(points)), (*C.int)(unsafe.Pointer(&faces[0])), C.int(len(faces)), C.double(voxelSize), C.double(bandwidth))
	return g
}

func (m *FloatGrid) Free() {
	C.vdb_float_grid_free(m.m)
	m.m = nil
}

func (m *FloatGrid) Clone() *FloatGrid {
	return &FloatGrid{
		m: C.vdb_float_grid_duplicate(m.m),
	}
}

func (m *FloatGrid) Read(file string) error {
	if m == nil || m.m == nil {
		return errors.New("Grid error ")
	}

	fname := C.CString(file)
	defer C.free(unsafe.Pointer(fname))
	C.vdb_float_grid_read(m.m, fname)

	return nil
}

func (m *FloatGrid) Write(file string) error {
	if m == nil || m.m == nil {
		return errors.New("Grid error ")
	}

	fname := C.CString(file)
	defer C.free(unsafe.Pointer(fname))
	C.vdb_float_grid_write(m.m, fname)

	return nil
}

func (m *FloatGrid) ToMesh() ([]float32, []int32, error) {
	if m == nil || m.m == nil {
		return nil, nil, errors.New("FloatGrid error ")
	}

	C.vdb_float_grid_to_mesh(m.m)

	var pointsLen C.int
	cpoints := C.vdb_float_grid_vertex_buffer(m.m, &pointsLen)

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
	cfaces := C.vdb_float_grid_face_buffer(m.m, &facesLen)

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

func (m *FloatGrid) ToMeshSettings(isovalue float64, adaptivity float64) ([]float32, []int32, error) {
	if m == nil || m.m == nil {
		return nil, nil, errors.New("FloatGrid error ")
	}
	C.vdb_float_grid_to_mesh_settings(m.m, C.double(isovalue), C.double(adaptivity))

	var pointsLen C.int
	cpoints := C.vdb_float_grid_vertex_buffer(m.m, &pointsLen)

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
	cfaces := C.vdb_float_grid_face_buffer(m.m, &facesLen)

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

func (m *FloatGrid) Transform(matrix []float64) (bool, error) {
	if m == nil || m.m == nil {
		return false, errors.New("FloatGrid error ")
	}
	ret := C.vdb_float_grid_transform(m.m, (*C.double)(unsafe.Pointer(&matrix[0])), C.int(len(matrix)))
	return bool(ret), nil
}

func (m *FloatGrid) BooleanUnion(csg *FloatGrid) error {
	if m == nil || m.m == nil {
		return errors.New("Union error ")
	}
	C.vdb_float_grid_union(m.m, csg.m)
	return nil
}

func (m *FloatGrid) BooleanDifference(csg *FloatGrid) error {
	if m == nil || m.m == nil {
		return errors.New("Difference error ")
	}
	C.vdb_float_grid_difference(m.m, csg.m)
	return nil
}

func (m *FloatGrid) BooleanIntersection(csg *FloatGrid) error {
	if m == nil || m.m == nil {
		return errors.New("Intersection error ")
	}
	C.vdb_float_grid_intersection(m.m, csg.m)
	return nil
}

func (m *FloatGrid) Offset(amount float64) error {
	if m == nil || m.m == nil {
		return errors.New("Offset error ")
	}
	C.vdb_float_grid_offset(m.m, C.double(amount))
	return nil
}

func (m *FloatGrid) OffsetMask(amount float64, mask *FloatGrid, min float64, max float64, invert bool) error {
	if m == nil || m.m == nil {
		return errors.New("OffsetMask error ")
	}
	C.vdb_float_grid_offset_mask(m.m, C.double(amount), mask.m, C.double(min), C.double(max), C.bool(invert))
	return nil
}

func (m *FloatGrid) Smooth(type_ SmoothType, iterations int32, width int32) error {
	if m == nil || m.m == nil {
		return errors.New("Smooth error ")
	}
	if width < 1 {
		width = 1
	}
	if iterations < 1 {
		iterations = 1
	}
	C.vdb_float_grid_smooth(m.m, C.int(type_), C.int(iterations), C.int(width))
	return nil
}

func (m *FloatGrid) SmoothMask(type_ SmoothType, iterations int32, width int32, mask *FloatGrid, min float64, max float64, invert bool) error {
	if m == nil || m.m == nil {
		return errors.New("SmoothMask error ")
	}
	if width < 1 {
		width = 1
	}
	if iterations < 1 {
		iterations = 1
	}
	C.vdb_float_grid_smooth_mask(m.m, C.int(type_), C.int(iterations), C.int(width), mask.m, C.double(min), C.double(max), C.bool(invert))
	return nil
}

func (m *FloatGrid) Blend(grid *FloatGrid, position float64, end float64) error {
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
	C.vdb_float_grid_blend(m.m, grid.m, C.double(position), C.double(end))
	return nil
}

func (m *FloatGrid) BlendMask(grid *FloatGrid, position float64, end float64, mask *FloatGrid, min float64, max float64, invert bool) error {
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
	C.vdb_float_grid_blend_mask(m.m, grid.m, C.double(position), C.double(end), mask.m, C.double(min), C.double(max), C.bool(invert))
	return nil
}

func (m *FloatGrid) ClosestPoint(points []float32) ([]float32, error) {
	if m == nil || m.m == nil {
		return nil, errors.New("ClosestPoint error ")
	}

	var pointsLen C.int
	cpoints := C.vdb_float_grid_closest_point(m.m, (*C.float)(unsafe.Pointer(&points[0])), C.int(len(points)), &pointsLen)

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

func (m *FloatGrid) Rebuild(iso float64, exWidth float64, inWidth float64) error {
	if m == nil || m.m == nil {
		return errors.New("Rebuild error ")
	}
	C.vdb_float_grid_rebuild(m.m, C.float(iso), C.float(exWidth), C.float(inWidth))
	return nil
}

func (m *FloatGrid) Dense() ([]float32, [3]int32, error) {
	if m == nil || m.m == nil {
		return nil, [3]int32{0, 0, 0}, errors.New("Rebuild error ")
	}

	var width C.int
	var height C.int
	var depth C.int
	cdense := C.vdb_float_grid_dense(m.m, &width, &height, &depth)
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

func (m *FloatGrid) Set(pos []int32, val float32) error {
	if m == nil || m.m == nil {
		return errors.New("Rebuild error ")
	}
	C.vdb_float_grid_set(m.m, C.int(pos[0]), C.int(pos[1]), C.int(pos[2]), C.float(val))
	return nil
}

func (m *FloatGrid) Get(pos []int32) (error, float32) {
	if m == nil || m.m == nil {
		return errors.New("Rebuild error "), float32(0.0)
	}
	ret := float32(C.vdb_float_grid_get(m.m, C.int(pos[0]), C.int(pos[1]), C.int(pos[2])))
	return nil, ret
}

func (m *FloatGrid) LinearGet(pos []float32) (error, float32) {
	if m == nil || m.m == nil {
		return errors.New("Rebuild error "), float32(0.0)
	}
	ret := float32(C.vdb_float_grid_linear_get(m.m, C.float(pos[0]), C.float(pos[1]), C.float(pos[2])))
	return nil, ret
}

func (m *FloatGrid) EvalActiveBoundingBox() (error, []float64) {
	if m == nil || m.m == nil {
		return errors.New("Rebuild error "), []float64{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
	}
	ret := make([]float64, 6)
	C.vdb_float_grid_eval_active_bounding_box(m.m, (*C.double)((unsafe.Pointer)(&ret[0])))
	return nil, ret
}

type PixelGrid struct {
	m *C.struct__vdb_pixel_grid_t
}

func NewPixelGrid() *PixelGrid {
	return &PixelGrid{
		m: C.vdb_pixel_grid_create(),
	}
}

func (m *PixelGrid) Free() {
	C.vdb_pixel_grid_free(m.m)
	m.m = nil
}

func (m *PixelGrid) Clone() *PixelGrid {
	return &PixelGrid{
		m: C.vdb_pixel_grid_duplicate(m.m),
	}
}

func (m *PixelGrid) Read(file string) error {
	if m == nil || m.m == nil {
		return errors.New("Grid error ")
	}

	fname := C.CString(file)
	defer C.free(unsafe.Pointer(fname))
	C.vdb_pixel_grid_read(m.m, fname)

	return nil
}

func (m *PixelGrid) Write(file string) error {
	if m == nil || m.m == nil {
		return errors.New("Grid error ")
	}

	fname := C.CString(file)
	defer C.free(unsafe.Pointer(fname))
	C.vdb_pixel_grid_write(m.m, fname)

	return nil
}

func (m *PixelGrid) Transform(matrix []float64) (bool, error) {
	if m == nil || m.m == nil {
		return false, errors.New("PixelGrid error ")
	}
	ret := C.vdb_pixel_grid_transform(m.m, (*C.double)(unsafe.Pointer(&matrix[0])), C.int(len(matrix)))
	return bool(ret), nil
}

type PixelType uint8

const (
	PT_COLOR             = PixelType(0)
	PT_MATERIAL          = PixelType(1)
	PT_MATERIAL_OR_COLOR = PixelType(2)
	PT_INVALID           = PixelType(3)
)

type Pixel struct {
	Type       PixelType
	MaterialId uint8
	FeatureId  uint16
	Color      [4]uint8
}

func pixel_2_cpixel(pixel Pixel) C.struct__vdb_pixel_t {
	var p C.struct__vdb_pixel_t
	p.tp = C.uchar(pixel.Type)
	p.material_id = C.uchar(pixel.MaterialId)
	p.feature_id = C.ushort(pixel.FeatureId)
	p.color_r = C.uchar(pixel.Color[0])
	p.color_g = C.uchar(pixel.Color[1])
	p.color_b = C.uchar(pixel.Color[2])
	p.color_a = C.uchar(pixel.Color[3])
	return p
}

func (m *PixelGrid) Set(pos []int32, val Pixel) error {
	if m == nil || m.m == nil {
		return errors.New("Rebuild error ")
	}
	C.vdb_pixel_grid_set(m.m, C.int(pos[0]), C.int(pos[1]), C.int(pos[2]), pixel_2_cpixel(val))
	return nil
}

func cpixel_2_pixel(cpixel C.struct__vdb_pixel_t) Pixel {
	var p Pixel
	p.Type = PixelType(cpixel.tp)
	p.MaterialId = uint8(cpixel.material_id)
	p.FeatureId = uint16(cpixel.feature_id)
	p.Color[0] = uint8(cpixel.color_r)
	p.Color[1] = uint8(cpixel.color_g)
	p.Color[2] = uint8(cpixel.color_b)
	p.Color[3] = uint8(cpixel.color_a)
	return p
}

func (m *PixelGrid) Get(pos []int32) (error, Pixel) {
	if m == nil || m.m == nil {
		return errors.New("Rebuild error "), Pixel{}
	}
	cpixel := C.vdb_pixel_grid_get(m.m, C.int(pos[0]), C.int(pos[1]), C.int(pos[2]))
	return nil, cpixel_2_pixel(cpixel)
}

func (m *PixelGrid) LinearGet(pos []float32) (error, Pixel) {
	if m == nil || m.m == nil {
		return errors.New("Rebuild error "), Pixel{}
	}
	cpixel := C.vdb_pixel_grid_linear_get(m.m, C.float(pos[0]), C.float(pos[1]), C.float(pos[2]))
	return nil, cpixel_2_pixel(cpixel)
}
