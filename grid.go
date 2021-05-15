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

	"github.com/flywave/flywave-vdb/coord"
	vec3d "github.com/flywave/go3d/float64/vec3"
	"github.com/flywave/go3d/vec3"
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

func (m *FloatGrid) GetTransform() *Transform {
	return &Transform{m: C.vdb_float_grid_get_transform(m.m)}
}

func (m *FloatGrid) SetTransform(tran *Transform) {
	C.vdb_float_grid_set_transform(m.m, tran.m)
}

func (m *FloatGrid) Empty() bool {
	return bool(C.vdb_float_grid_empty(m.m))
}

func (m *FloatGrid) Clear() {
	C.vdb_float_grid_clear(m.m)
}

func (m *FloatGrid) GetBackground() float32 {
	return float32(C.vdb_float_grid_get_background(m.m))
}

func (m *FloatGrid) IsSaveFloatAsHalf() bool {
	return bool(C.vdb_float_grid_save_float_as_half(m.m))
}

func (m *FloatGrid) SetSaveFloatAsHalf(v bool) {
	C.vdb_float_grid_set_save_float_as_half(m.m, C.bool(v))
}

func (m *FloatGrid) GetActiveVoxelCount() uint64 {
	return uint64(C.vdb_float_grid_active_voxel_count(m.m))
}

func (m *FloatGrid) Prune(tolerance float32) {
	C.vdb_float_grid_prune(m.m, C.float(tolerance))
}

func (m *FloatGrid) Clip(bbox vec3d.T) {
	C.vdb_float_grid_clip(m.m, (*C.double)((unsafe.Pointer)(&bbox.Slice()[0])))
}

func (m *FloatGrid) ClipFromCoordbox(cbox coord.Box) {
	C.vdb_float_grid_clip_from_coordbox(m.m, (*C.int)(cbox.CSlice()))
}

func (m *FloatGrid) GetActiveVoxelBoundingBox() *coord.Box {
	cbox := make([]int32, 6)
	C.vdb_float_grid_active_voxel_bounding_box(m.m, (*C.int)((unsafe.Pointer)(&cbox[0])))
	return coord.FromSlice(cbox)
}

func (m *FloatGrid) GetActiveVoxelDim() coord.T {
	dim := coord.T{}
	C.vdb_float_grid_active_voxel_dim(m.m, (*C.int)((unsafe.Pointer)(&dim[0])))
	return dim
}

func (m *FloatGrid) PrintInfo() string {
	cstr := C.vdb_float_grid_print_info(m.m)
	defer C.free((unsafe.Pointer)(cstr))
	return C.GoString(cstr)
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

func (m *FloatGrid) ClosestPoint(points []vec3.T) ([]vec3.T, error) {
	if m == nil || m.m == nil {
		return nil, errors.New("ClosestPoint error ")
	}

	var pointsLen C.int
	cpoints := C.vdb_float_grid_closest_point(m.m, (*C.float)(unsafe.Pointer(&points[0])), C.int(len(points)*3), &pointsLen)

	var cpointsSlice []float32
	cpointsHeader := (*reflect.SliceHeader)((unsafe.Pointer(&cpointsSlice)))
	cpointsHeader.Cap = int(pointsLen)
	cpointsHeader.Len = int(pointsLen)
	cpointsHeader.Data = uintptr(unsafe.Pointer(cpoints))

	closest_points := make([]vec3.T, int(pointsLen)/3)
	for i := 0; i < int(pointsLen)/3; i++ {
		closest_points[i] = vec3.T{cpointsSlice[i*3], cpointsSlice[i*3+1], cpointsSlice[i*3+2]}
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

func (m *FloatGrid) Dense() ([]float32, coord.T, error) {
	if m == nil || m.m == nil {
		return nil, coord.T{0, 0, 0}, errors.New("Rebuild error ")
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
	return dense, coord.T{int32(width), int32(height), int32(depth)}, nil
}

func (m *FloatGrid) Set(pos coord.T, val float32) error {
	if m == nil || m.m == nil {
		return errors.New("Rebuild error ")
	}
	C.vdb_float_grid_set(m.m, C.int(pos[0]), C.int(pos[1]), C.int(pos[2]), C.float(val))
	return nil
}

func (m *FloatGrid) Get(pos coord.T) (error, float32) {
	if m == nil || m.m == nil {
		return errors.New("Rebuild error "), float32(0.0)
	}
	ret := float32(C.vdb_float_grid_get(m.m, C.int(pos[0]), C.int(pos[1]), C.int(pos[2])))
	return nil, ret
}

func (m *FloatGrid) LinearGet(pos vec3.T) (error, float32) {
	if m == nil || m.m == nil {
		return errors.New("Rebuild error "), float32(0.0)
	}
	ret := float32(C.vdb_float_grid_linear_get(m.m, C.float(pos[0]), C.float(pos[1]), C.float(pos[2])))
	return nil, ret
}

func (m *FloatGrid) EvalActiveBoundingBox() (error, *vec3d.Box) {
	if m == nil || m.m == nil {
		return errors.New("Rebuild error "), nil
	}
	ret := make([]float64, 6)
	C.vdb_float_grid_eval_active_bounding_box(m.m, (*C.double)((unsafe.Pointer)(&ret[0])))
	return nil, &vec3d.Box{Min: vec3d.T{ret[0], ret[1], ret[2]}, Max: vec3d.T{ret[3], ret[4], ret[5]}}
}

func (m *FloatGrid) ResampleWithRef(ref *FloatGrid, voxelSize float32,
	curOrder SamplerType, tolerance float32, prune bool) *FloatGrid {
	refm := (*C.struct__vdb_float_grid_t)(nil)
	if ref != nil {
		refm = ref.m
	}

	return &FloatGrid{m: C.vdb_float_grid_resample_with_ref(m.m, refm, C.float(voxelSize), C.int(curOrder), C.float(tolerance), C.bool(prune))}
}

func (m *FloatGrid) ResampleWithTransform(tran GridTransform,
	curOrder SamplerType, tolerance float32, prune bool) *FloatGrid {
	trana := NewGridTransformAdapter(tran)
	defer trana.Free()
	return &FloatGrid{m: C.vdb_float_grid_resample_with_grid_transform(m.m, trana.m, C.int(curOrder), C.float(tolerance), C.bool(prune))}
}

func (m *FloatGrid) SparseFill(box *coord.Box, v float32, active bool) {
	C.vdb_float_grid_sparse_fill(m.m, (*C.int)(box.CSlice()), C.float(v), C.bool(active))
}

func (m *FloatGrid) Fill(box *coord.Box, v float32, active bool) {
	C.vdb_float_grid_fill(m.m, (*C.int)(box.CSlice()), C.float(v), C.bool(active))
}

func (m *FloatGrid) DenseFill(box *coord.Box, v float32, active bool) {
	C.vdb_float_grid_dense_fill(m.m, (*C.int)(box.CSlice()), C.float(v), C.bool(active))
}

//export vdbFloatGridVisiton
func vdbFloatGridVisiton(ctx unsafe.Pointer, coord_ *C.int, val C.float) C.bool {
	var aSlice []int32
	aHeader := (*reflect.SliceHeader)((unsafe.Pointer(&aSlice)))
	aHeader.Cap = int(3)
	aHeader.Len = int(3)
	aHeader.Data = uintptr(unsafe.Pointer(coord_))

	ccoord := coord.T{aSlice[0], aSlice[1], aSlice[2]}

	return C.bool((*(*func(coord coord.T, v float32) bool)(ctx))(ccoord, float32(val)))
}

func (m *FloatGrid) VisitOn(v func(coord coord.T, v float32) bool) {
	inptr := uintptr(unsafe.Pointer(&v))
	C.vdb_float_grid_visit_on(m.m, (unsafe.Pointer)(inptr))
}

func (m *FloatGrid) VisitOff(v func(coord coord.T, v float32) bool) {
	inptr := uintptr(unsafe.Pointer(&v))
	C.vdb_float_grid_visit_off(m.m, (unsafe.Pointer)(inptr))
}

func (m *FloatGrid) VisitAll(v func(coord coord.T, v float32) bool) {
	inptr := uintptr(unsafe.Pointer(&v))
	C.vdb_float_grid_visit_all(m.m, (unsafe.Pointer)(inptr))
}

type FloatGridIterator struct {
	m *C.struct__vdb_float_grid_iterator_t
}

func (m *FloatGridIterator) SetValue(v float32) {
	C.vdb_float_grid_iterator_set_value(m.m, C.float(v))
}

func (m *FloatGridIterator) GetValue() float32 {
	return float32(C.vdb_float_grid_iterator_get_value(m.m))
}

func (m *FloatGridIterator) SetActiveState(v bool) {
	C.vdb_float_grid_iterator_set_active_state(m.m, C.bool(v))
}

func (m *FloatGridIterator) SetValueOff() {
	C.vdb_float_grid_iterator_set_value_off(m.m)
}

func (m *FloatGridIterator) SetMinDepth(v int32) {
	C.vdb_float_grid_iterator_set_min_depth(m.m, C.int(v))
}

func (m *FloatGridIterator) GetMinDepth() int32 {
	return int32(C.vdb_float_grid_iterator_get_min_depth(m.m))
}

func (m *FloatGridIterator) SetMaxDepth(v int32) {
	C.vdb_float_grid_iterator_set_max_depth(m.m, C.int(v))
}

func (m *FloatGridIterator) GetMaxDepth() int32 {
	return int32(C.vdb_float_grid_iterator_get_max_depth(m.m))
}

func (m *FloatGridIterator) Test() bool {
	return bool(C.vdb_float_grid_iterator_test(m.m))
}

func (m *FloatGridIterator) IsTileValue() bool {
	return bool(C.vdb_float_grid_iterator_is_tile_value(m.m))
}

func (m *FloatGridIterator) IsVoxelValue() bool {
	return bool(C.vdb_float_grid_iterator_is_voxel_value(m.m))
}

func (m *FloatGridIterator) IsValueOn() bool {
	return bool(C.vdb_float_grid_iterator_is_value_on(m.m))
}

func (m *FloatGridIterator) GetLevel() int32 {
	return int32(C.vdb_float_grid_iterator_get_level(m.m))
}

func (m *FloatGridIterator) GetDepth() int32 {
	return int32(C.vdb_float_grid_iterator_get_depth(m.m))
}

func (m *FloatGridIterator) GetLeafDepth() int32 {
	return int32(C.vdb_float_grid_iterator_get_leaf_depth(m.m))
}

func (m *FloatGridIterator) GetCoord() coord.T {
	ccoord := make([]int32, 3)
	C.vdb_float_grid_iterator_get_coord(m.m, (*C.int)(unsafe.Pointer(&ccoord[0])))
	return coord.T{ccoord[0], ccoord[1], ccoord[2]}
}

func (m *FloatGridIterator) GetCoordBox() *coord.Box {
	cb := make([]int32, 6)
	C.vdb_float_grid_iterator_get_bounding_box(m.m, (*C.int)(unsafe.Pointer(&cb[0])))
	return coord.FromSlice(cb)
}

func (m *FloatGridIterator) GetVoxelCount() int32 {
	return int32(C.vdb_float_grid_iterator_get_voxel_count(m.m))
}

func (m *FloatGrid) VisitOnIterator(v func(iter *FloatGridIterator) bool) {
	inptr := uintptr(unsafe.Pointer(&v))
	C.vdb_float_grid_visit_iterator_on(m.m, (unsafe.Pointer)(inptr))
}

func (m *FloatGrid) VisitOffIterator(v func(iter *FloatGridIterator) bool) {
	inptr := uintptr(unsafe.Pointer(&v))
	C.vdb_float_grid_visit_iterator_off(m.m, (unsafe.Pointer)(inptr))
}

func (m *FloatGrid) VisitAllIterator(v func(iter *FloatGridIterator) bool) {
	inptr := uintptr(unsafe.Pointer(&v))
	C.vdb_float_grid_visit_iterator_all(m.m, (unsafe.Pointer)(inptr))
}

//export vdbFloatGridVisitonIterator
func vdbFloatGridVisitonIterator(ctx unsafe.Pointer, iter *C.struct__vdb_float_grid_iterator_t) C.bool {
	return C.bool((*(*func(iter FloatGridIterator) bool)(ctx))(FloatGridIterator{m: iter}))
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

func (m *PixelGrid) GetTransform() *Transform {
	return &Transform{m: C.vdb_pixel_grid_get_transform(m.m)}
}

func (m *PixelGrid) SetTransform(tran *Transform) {
	C.vdb_pixel_grid_set_transform(m.m, tran.m)
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

func (m *PixelGrid) Set(pos coord.T, val Pixel) error {
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

func (m *PixelGrid) Get(pos coord.T) (error, Pixel) {
	if m == nil || m.m == nil {
		return errors.New("Rebuild error "), Pixel{}
	}
	cpixel := C.vdb_pixel_grid_get(m.m, C.int(pos[0]), C.int(pos[1]), C.int(pos[2]))
	return nil, cpixel_2_pixel(cpixel)
}

func (m *PixelGrid) LinearGet(pos vec3.T) (error, Pixel) {
	if m == nil || m.m == nil {
		return errors.New("Rebuild error "), Pixel{}
	}
	cpixel := C.vdb_pixel_grid_linear_get(m.m, C.float(pos[0]), C.float(pos[1]), C.float(pos[2]))
	return nil, cpixel_2_pixel(cpixel)
}

func (m *PixelGrid) Empty() bool {
	return bool(C.vdb_pixel_grid_empty(m.m))
}

func (m *PixelGrid) Clear() {
	C.vdb_pixel_grid_clear(m.m)
}

func (m *PixelGrid) GetActiveVoxelCount() uint64 {
	return uint64(C.vdb_pixel_grid_active_voxel_count(m.m))
}

func (m *PixelGrid) Prune(tolerance float32) {
	C.vdb_pixel_grid_prune(m.m, C.float(tolerance))
}

func (m *PixelGrid) Clip(bbox vec3d.Box) {
	C.vdb_pixel_grid_clip(m.m, (*C.double)((unsafe.Pointer)(&bbox.Slice()[0])))
}

func (m *PixelGrid) ClipFromCoordbox(cbox coord.Box) {
	C.vdb_pixel_grid_clip_from_coordbox(m.m, (*C.int)(cbox.CSlice()))
}

func (m *PixelGrid) GetActiveVoxelBoundingBox() *coord.Box {
	cbox := make([]int32, 6)
	C.vdb_pixel_grid_active_voxel_bounding_box(m.m, (*C.int)((unsafe.Pointer)(&cbox[0])))
	return coord.FromSlice(cbox)
}

func (m *PixelGrid) GetActiveVoxelDim() coord.T {
	dim := make([]int32, 3)
	C.vdb_pixel_grid_active_voxel_dim(m.m, (*C.int)((unsafe.Pointer)(&dim[0])))
	return coord.T{dim[0], dim[1], dim[2]}
}

func (m *PixelGrid) PrintInfo() string {
	cstr := C.vdb_pixel_grid_print_info(m.m)
	defer C.free((unsafe.Pointer)(cstr))
	return C.GoString(cstr)
}

//export vdbPixelGridVisiton
func vdbPixelGridVisiton(ctx unsafe.Pointer, c *C.int, val C.struct__vdb_pixel_t) C.bool {
	var aSlice []int32
	aHeader := (*reflect.SliceHeader)((unsafe.Pointer(&aSlice)))
	aHeader.Cap = int(3)
	aHeader.Len = int(3)
	aHeader.Data = uintptr(unsafe.Pointer(c))

	ccoord := coord.T{aSlice[0], aSlice[1], aSlice[2]}

	return C.bool((*(*func(c coord.T, v Pixel) bool)(ctx))(ccoord, cpixel_2_pixel(val)))
}

func (m *PixelGrid) VisitOn(v func(c coord.T, v Pixel) bool) {
	inptr := uintptr(unsafe.Pointer(&v))
	C.vdb_pixel_grid_visit_on(m.m, (unsafe.Pointer)(inptr))
}

func (m *PixelGrid) VisitOff(v func(c coord.T, v Pixel) bool) {
	inptr := uintptr(unsafe.Pointer(&v))
	C.vdb_pixel_grid_visit_off(m.m, (unsafe.Pointer)(inptr))
}

func (m *PixelGrid) VisitAll(v func(c coord.T, v Pixel) bool) {
	inptr := uintptr(unsafe.Pointer(&v))
	C.vdb_pixel_grid_visit_all(m.m, (unsafe.Pointer)(inptr))
}

func (m *PixelGrid) SparseFill(box *coord.Box, v Pixel, active bool) {
	C.vdb_pixel_grid_sparse_fill(m.m, (*C.int)(box.CSlice()), pixel_2_cpixel(v), C.bool(active))
}

func (m *PixelGrid) Fill(box *coord.Box, v Pixel, active bool) {
	C.vdb_pixel_grid_fill(m.m, (*C.int)(box.CSlice()), pixel_2_cpixel(v), C.bool(active))
}

func (m *PixelGrid) DenseFill(box *coord.Box, v Pixel, active bool) {
	C.vdb_pixel_grid_dense_fill(m.m, (*C.int)(box.CSlice()), pixel_2_cpixel(v), C.bool(active))
}

type PixelGridIterator struct {
	m *C.struct__vdb_pixel_grid_iterator_t
}

func (m *PixelGridIterator) SetValue(v Pixel) {
	C.vdb_pixel_grid_iterator_set_value(m.m, pixel_2_cpixel(v))
}

func (m *PixelGridIterator) GetValue() Pixel {
	return cpixel_2_pixel(C.vdb_pixel_grid_iterator_get_value(m.m))
}

func (m *PixelGridIterator) SetActiveState(v bool) {
	C.vdb_pixel_grid_iterator_set_active_state(m.m, C.bool(v))
}

func (m *PixelGridIterator) SetValueOff() {
	C.vdb_pixel_grid_iterator_set_value_off(m.m)
}

func (m *PixelGridIterator) SetMinDepth(v int32) {
	C.vdb_pixel_grid_iterator_set_min_depth(m.m, C.int(v))
}

func (m *PixelGridIterator) GetMinDepth() int32 {
	return int32(C.vdb_pixel_grid_iterator_get_min_depth(m.m))
}

func (m *PixelGridIterator) SetMaxDepth(v int32) {
	C.vdb_pixel_grid_iterator_set_max_depth(m.m, C.int(v))
}

func (m *PixelGridIterator) GetMaxDepth() int32 {
	return int32(C.vdb_pixel_grid_iterator_get_max_depth(m.m))
}

func (m *PixelGridIterator) Test() bool {
	return bool(C.vdb_pixel_grid_iterator_test(m.m))
}

func (m *PixelGridIterator) IsTileValue() bool {
	return bool(C.vdb_pixel_grid_iterator_is_tile_value(m.m))
}

func (m *PixelGridIterator) IsVoxelValue() bool {
	return bool(C.vdb_pixel_grid_iterator_is_voxel_value(m.m))
}

func (m *PixelGridIterator) IsValueOn() bool {
	return bool(C.vdb_pixel_grid_iterator_is_value_on(m.m))
}

func (m *PixelGridIterator) GetLevel() int32 {
	return int32(C.vdb_pixel_grid_iterator_get_level(m.m))
}

func (m *PixelGridIterator) GetDepth() int32 {
	return int32(C.vdb_pixel_grid_iterator_get_depth(m.m))
}

func (m *PixelGridIterator) GetLeafDepth() int32 {
	return int32(C.vdb_pixel_grid_iterator_get_leaf_depth(m.m))
}

func (m *PixelGridIterator) GetCoord() coord.T {
	c := make([]int32, 3)
	C.vdb_pixel_grid_iterator_get_coord(m.m, (*C.int)(unsafe.Pointer(&c[0])))
	return coord.T{c[0], c[1], c[2]}
}

func (m *PixelGridIterator) GetCoordBox() *coord.Box {
	cb := make([]int32, 6)
	C.vdb_pixel_grid_iterator_get_bounding_box(m.m, (*C.int)(unsafe.Pointer(&cb[0])))
	return coord.FromSlice(cb)
}

func (m *PixelGridIterator) GetVoxelCount() int32 {
	return int32(C.vdb_pixel_grid_iterator_get_voxel_count(m.m))
}

func (m *PixelGrid) VisitOnIterator(v func(iter *PixelGridIterator) bool) {
	inptr := uintptr(unsafe.Pointer(&v))
	C.vdb_pixel_grid_visit_iterator_on(m.m, (unsafe.Pointer)(inptr))
}

func (m *PixelGrid) VisitOffIterator(v func(iter *PixelGridIterator) bool) {
	inptr := uintptr(unsafe.Pointer(&v))
	C.vdb_pixel_grid_visit_iterator_off(m.m, (unsafe.Pointer)(inptr))
}

func (m *PixelGrid) VisitAllIterator(v func(iter *PixelGridIterator) bool) {
	inptr := uintptr(unsafe.Pointer(&v))
	C.vdb_pixel_grid_visit_iterator_all(m.m, (unsafe.Pointer)(inptr))
}

//export vdbPixelGridVisitonIterator
func vdbPixelGridVisitonIterator(ctx unsafe.Pointer, iter *C.struct__vdb_pixel_grid_iterator_t) C.bool {
	return C.bool((*(*func(iter PixelGridIterator) bool)(ctx))(PixelGridIterator{m: iter}))
}
