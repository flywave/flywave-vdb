package vdb

// #include <stdlib.h>
// #include "voxelizer_api.h"
// #cgo CFLAGS: -I ./lib
// #cgo CXXFLAGS: -I ./lib
import "C"
import (
	"errors"
	"reflect"
	"unsafe"
)

func init() {
	C.voxel_pixel_initialize()
}

type VoxelPixel struct {
	m *C.struct__voxel_pixel_t
}

func NewVoxelPixel() *VoxelPixel {
	return &VoxelPixel{
		m: C.voxel_pixel_create(),
	}
}

func (m *VoxelPixel) Free() {
	C.voxel_pixel_free(m.m)
	m.m = nil
}

func (t *VoxelPixel) Vaild() bool {
	return t.m != nil
}

func (t *VoxelPixel) Size() int64 {
	return int64(C.voxel_pixel_get_memory_size(t.m))
}

func (m *VoxelPixel) Clone() *VoxelPixel {
	return &VoxelPixel{
		m: C.voxel_pixel_duplicate(m.m),
	}
}

func (m *VoxelPixel) Read(file string) error {
	if m == nil || m.m == nil {
		return errors.New("Grid error!")
	}

	fname := C.CString(file)
	defer C.free(unsafe.Pointer(fname))
	C.voxel_pixel_read(m.m, fname)

	return nil
}

func (m *VoxelPixel) Write(file string) error {
	if m == nil || m.m == nil {
		return errors.New("Grid error!")
	}

	fname := C.CString(file)
	defer C.free(unsafe.Pointer(fname))
	C.voxel_pixel_write(m.m, fname)

	return nil
}

func (m *VoxelPixel) Composite(t *VoxelPixel, tp CompositeType) {
	C.voxel_pixel_composite(m.m, t.m, C.uint(tp))
}

func (m *VoxelPixel) Clear() {
	C.voxel_pixel_clear(m.m)
}

func (m *VoxelPixel) ClearUnuseMaterials() {
	C.voxel_pixel_clear_unuse_materials(m.m)
}

func (m *VoxelPixel) ClearUnuseFeatures() {
	C.voxel_pixel_clear_unuse_features(m.m)
}

func (m *VoxelPixel) ClearMaterials() {
	C.voxel_pixel_clear_materials(m.m)
}

func (m *VoxelPixel) ClearFeatures() {
	C.voxel_pixel_clear_features(m.m)
}

func (m *VoxelPixel) RayTest(r *Ray) (bool, []float64) {
	p := make([]float64, 3)
	ret := bool(C.voxel_pixel_ray_test(m.m, (*C.double)((unsafe.Pointer)(&r.origin[0])), (*C.double)((unsafe.Pointer)(&r.dir[0])), (*C.double)((unsafe.Pointer)(&p[0]))))
	return ret, p
}

func (m *VoxelPixel) RayTests(rs []Ray) [][]float64 {
	origin := make([]float64, 3*len(rs))
	dir := make([]float64, 3*len(rs))
	for i, _ := range rs {
		origin[(i * 3)] = rs[i].origin[0]
		origin[(i*3)+1] = rs[i].origin[1]
		origin[(i*3)+2] = rs[i].origin[2]

		dir[(i * 3)] = rs[i].dir[0]
		dir[(i*3)+1] = rs[i].dir[1]
		dir[(i*3)+2] = rs[i].dir[2]
	}
	p := make([]float64, 3*len(rs))
	ok := bool(C.voxel_pixel_ray_tests(m.m, (*C.double)((unsafe.Pointer)(&origin[0])), (*C.double)((unsafe.Pointer)(&dir[0])), (*C.double)((unsafe.Pointer)(&p[0])), C.size_t(len(rs))))
	if !ok {
		return nil
	}
	pos := make([][]float64, len(rs))
	for i := 0; i < len(rs); i++ {
		pos[i] = p[i*3 : (i*3)+2]
	}
	return pos
}

func (m *VoxelPixel) VoxelResolution() *Transform {
	return &Transform{m: C.voxel_pixel_voxel_resolution(m.m)}
}

func (m *VoxelPixel) GetVoxelGrid() *FloatGrid {
	return &FloatGrid{m: C.voxel_pixel_get_voxel_grid(m.m)}
}

func (m *VoxelPixel) GetPixelGrid() *PixelGrid {
	return &PixelGrid{m: C.voxel_pixel_get_pixel_grid(m.m)}
}

func (m *VoxelPixel) SetVoxelGrid(g *FloatGrid) {
	C.voxel_pixel_set_voxel_grid(m.m, g.m)
}

func (m *VoxelPixel) SetPixelGrid(g *PixelGrid) {
	C.voxel_pixel_set_pixel_grid(m.m, g.m)
}

func (m *VoxelPixel) Empty() bool {
	return bool(C.voxel_pixel_is_empty(m.m))
}

func (m *VoxelPixel) GetMaterials() *Materials {
	return &Materials{m: C.voxel_pixel_get_materials(m.m)}
}

func (m *VoxelPixel) SetMaterials(mtl *Materials) {
	C.voxel_pixel_set_materials(m.m, mtl.m)
}

func (m *VoxelPixel) AddMaterial(mtl *MaterialData) {
	C.voxel_pixel_add_material(m.m, mtl.m)
}

func (m *VoxelPixel) RemoveMaterial(id uint8) {
	C.voxel_pixel_remove_material(m.m, C.uchar(id))
}

func (m *VoxelPixel) HasMaterial(id uint8) bool {
	return bool(C.voxel_pixel_has_material(m.m, C.uchar(id)))
}

func (m *VoxelPixel) MaterialCount() int {
	return int(C.voxel_pixel_materials_count(m.m))
}

func (m *VoxelPixel) GetFeatures() *Features {
	return &Features{m: C.voxel_pixel_get_features(m.m)}
}

func (m *VoxelPixel) SetFeatures(feats *Features) {
	C.voxel_pixel_set_features(m.m, feats.m)
}

func (m *VoxelPixel) AddFeature(feat *FeatureData) {
	C.voxel_pixel_add_feature(m.m, feat.m)
}

func (m *VoxelPixel) RemoveFeature(id uint16) {
	C.voxel_pixel_remove_feature(m.m, C.ushort(id))
}

func (m *VoxelPixel) HasFeature(id uint16) bool {
	return bool(C.voxel_pixel_has_feature(m.m, C.ushort(id)))
}

func (m *VoxelPixel) FeatureCount() int {
	return int(C.voxel_pixel_features_count(m.m))
}

func (m *VoxelPixel) ExtractColor(src *VoxelPixel) *PixelGrid {
	return &PixelGrid{m: C.voxel_pixel_extract_color(m.m, src.m)}
}

func (m *VoxelPixel) FillColor(src *VoxelPixel, color *PixelGrid) {
	C.voxel_pixel_fill_color(m.m, src.m, color.m)
}

func (m *VoxelPixel) EvalMaxMinElevation(in *BBox) *BBox {
	var out *BBox
	C.voxel_pixel_eval_max_min_elevation(m.m, &in.m[0], &out.m[0])
	return out
}

type Vertex struct {
	V [3]float32
	C [4]uint8
	T [2]float32
	B bool
}

type Triangle struct {
	V    [3]Vertex
	Node uint32
	Tex  uint32
	Mtl  uint32
	FID  uint64
}

func (m *VoxelPixel) MakeTriangles(mat []float64, texOffset int, mtlOffset int, bl BorderLock, ft FilterTriangle, fquality float64,
	isovalue float64, adapter float64) []Triangle {
	var tricount C.size_t
	var ctris *C.struct__voxel_io_triangle_t
	cbl := NewBorderLockAdapter(bl)
	defer cbl.Free()
	cft := NewFilterTriangleAdapter(ft)
	defer cft.Free()

	C.voxel_pixel_make_triangles(m.m, &ctris, &tricount, (*C.double)((unsafe.Pointer)(&mat[0])), C.size_t(texOffset), C.size_t(mtlOffset), cbl.m, cft.m, C.double(fquality), C.double(isovalue), C.double(adapter))
	defer C.free(unsafe.Pointer(ctris))

	var trisSlice []C.struct__voxel_io_triangle_t
	trisHeader := (*reflect.SliceHeader)((unsafe.Pointer(&trisSlice)))
	trisHeader.Cap = int(tricount)
	trisHeader.Len = int(tricount)
	trisHeader.Data = uintptr(unsafe.Pointer(ctris))

	ret := make([]Triangle, int(tricount))

	for i := 0; i < int(tricount); i++ {
		ret[i].V[0].V[0] = float32(trisSlice[i].v_a.v_x)
		ret[i].V[0].V[1] = float32(trisSlice[i].v_a.v_y)
		ret[i].V[0].V[2] = float32(trisSlice[i].v_a.v_z)

		ret[i].V[0].C[0] = uint8(trisSlice[i].v_a.c_r)
		ret[i].V[0].C[1] = uint8(trisSlice[i].v_a.c_g)
		ret[i].V[0].C[2] = uint8(trisSlice[i].v_a.c_b)
		ret[i].V[0].C[3] = uint8(trisSlice[i].v_a.c_a)

		ret[i].V[0].T[0] = float32(trisSlice[i].v_a.t_u)
		ret[i].V[0].T[1] = float32(trisSlice[i].v_a.t_v)

		ret[i].V[0].B = bool(trisSlice[i].v_a.b)

		ret[i].V[1].V[0] = float32(trisSlice[i].v_b.v_x)
		ret[i].V[1].V[1] = float32(trisSlice[i].v_b.v_y)
		ret[i].V[1].V[2] = float32(trisSlice[i].v_b.v_z)

		ret[i].V[1].C[0] = uint8(trisSlice[i].v_b.c_r)
		ret[i].V[1].C[1] = uint8(trisSlice[i].v_b.c_g)
		ret[i].V[1].C[2] = uint8(trisSlice[i].v_b.c_b)
		ret[i].V[1].C[3] = uint8(trisSlice[i].v_b.c_a)

		ret[i].V[1].T[0] = float32(trisSlice[i].v_b.t_u)
		ret[i].V[1].T[1] = float32(trisSlice[i].v_b.t_v)

		ret[i].V[1].B = bool(trisSlice[i].v_b.b)

		ret[i].V[2].V[0] = float32(trisSlice[i].v_c.v_x)
		ret[i].V[2].V[1] = float32(trisSlice[i].v_c.v_y)
		ret[i].V[2].V[2] = float32(trisSlice[i].v_c.v_z)

		ret[i].V[2].C[0] = uint8(trisSlice[i].v_c.c_r)
		ret[i].V[2].C[1] = uint8(trisSlice[i].v_c.c_g)
		ret[i].V[2].C[2] = uint8(trisSlice[i].v_c.c_b)
		ret[i].V[2].C[3] = uint8(trisSlice[i].v_c.c_a)

		ret[i].V[2].T[0] = float32(trisSlice[i].v_c.t_u)
		ret[i].V[2].T[1] = float32(trisSlice[i].v_c.t_v)

		ret[i].V[2].B = bool(trisSlice[i].v_c.b)

		ret[i].Node = uint32(trisSlice[i].node)
		ret[i].Tex = uint32(trisSlice[i].tex)
		ret[i].Mtl = uint32(trisSlice[i].mtl)
		ret[i].FID = uint64(trisSlice[i].feature_id)
	}

	return ret
}

func (m *VoxelPixel) MakeTrianglesSimple(mat []float64, texOffset int, mtlOffset int, fquality float64,
	isovalue float64, adapter float64) []Triangle {
	var tricount C.size_t
	var ctris *C.struct__voxel_io_triangle_t
	C.voxel_pixel_make_triangles_simple(m.m, &ctris, &tricount, (*C.double)((unsafe.Pointer)(&mat[0])), C.size_t(texOffset), C.size_t(mtlOffset), C.double(fquality), C.double(isovalue), C.double(adapter))
	defer C.free(unsafe.Pointer(ctris))

	var trisSlice []C.struct__voxel_io_triangle_t
	trisHeader := (*reflect.SliceHeader)((unsafe.Pointer(&trisSlice)))
	trisHeader.Cap = int(tricount)
	trisHeader.Len = int(tricount)
	trisHeader.Data = uintptr(unsafe.Pointer(ctris))

	ret := make([]Triangle, int(tricount))

	for i := 0; i < int(tricount); i++ {
		ret[i].V[0].V[0] = float32(trisSlice[i].v_a.v_x)
		ret[i].V[0].V[1] = float32(trisSlice[i].v_a.v_y)
		ret[i].V[0].V[2] = float32(trisSlice[i].v_a.v_z)

		ret[i].V[0].C[0] = uint8(trisSlice[i].v_a.c_r)
		ret[i].V[0].C[1] = uint8(trisSlice[i].v_a.c_g)
		ret[i].V[0].C[2] = uint8(trisSlice[i].v_a.c_b)
		ret[i].V[0].C[3] = uint8(trisSlice[i].v_a.c_a)

		ret[i].V[0].T[0] = float32(trisSlice[i].v_a.t_u)
		ret[i].V[0].T[1] = float32(trisSlice[i].v_a.t_v)

		ret[i].V[0].B = bool(trisSlice[i].v_a.b)

		ret[i].V[1].V[0] = float32(trisSlice[i].v_b.v_x)
		ret[i].V[1].V[1] = float32(trisSlice[i].v_b.v_y)
		ret[i].V[1].V[2] = float32(trisSlice[i].v_b.v_z)

		ret[i].V[1].C[0] = uint8(trisSlice[i].v_b.c_r)
		ret[i].V[1].C[1] = uint8(trisSlice[i].v_b.c_g)
		ret[i].V[1].C[2] = uint8(trisSlice[i].v_b.c_b)
		ret[i].V[1].C[3] = uint8(trisSlice[i].v_b.c_a)

		ret[i].V[1].T[0] = float32(trisSlice[i].v_b.t_u)
		ret[i].V[1].T[1] = float32(trisSlice[i].v_b.t_v)

		ret[i].V[1].B = bool(trisSlice[i].v_b.b)

		ret[i].V[2].V[0] = float32(trisSlice[i].v_c.v_x)
		ret[i].V[2].V[1] = float32(trisSlice[i].v_c.v_y)
		ret[i].V[2].V[2] = float32(trisSlice[i].v_c.v_z)

		ret[i].V[2].C[0] = uint8(trisSlice[i].v_c.c_r)
		ret[i].V[2].C[1] = uint8(trisSlice[i].v_c.c_g)
		ret[i].V[2].C[2] = uint8(trisSlice[i].v_c.c_b)
		ret[i].V[2].C[3] = uint8(trisSlice[i].v_c.c_a)

		ret[i].V[2].T[0] = float32(trisSlice[i].v_c.t_u)
		ret[i].V[2].T[1] = float32(trisSlice[i].v_c.t_v)

		ret[i].V[2].B = bool(trisSlice[i].v_c.b)

		ret[i].Node = uint32(trisSlice[i].node)
		ret[i].Tex = uint32(trisSlice[i].tex)
		ret[i].Mtl = uint32(trisSlice[i].mtl)
		ret[i].FID = uint64(trisSlice[i].feature_id)
	}

	return ret
}

func (m *VoxelPixel) MakeAtlasGenerator(mat []float64, pixelPad float32) *AtlasGenerator {
	return &AtlasGenerator{m: C.voxel_pixel_make_texture_atlas_generator(m.m, (*C.double)((unsafe.Pointer)(&mat[0])), C.float(pixelPad))}
}
