package vdb

// #include <stdlib.h>
// #include "voxelizer_api.h"
// #cgo CFLAGS: -I ./lib
// #cgo CXXFLAGS: -I ./lib
import "C"
import "unsafe"

type Axis int32

const (
	X_AXIS = Axis(0)
	Y_AXIS = Axis(1)
	Z_AXIS = Axis(2)
)

type Transform struct {
	m *C.struct__voxel_transform_t
}

func NewTransformFromVoxelSize(voxelSize float64) (error, *Transform) {
	return nil, &Transform{m: C.voxel_transform_create_from_voxel_size(C.double(voxelSize))}
}

func NewTransformFromMat4d(mat []float64) (error, *Transform) {
	return nil, &Transform{m: C.voxel_transform_create_from_mat((*C.double)((unsafe.Pointer)(&mat[0])))}
}

func NewTransformFromFrustum(bbox *BBox, taper float64, depth float64, voxelSize float64) (error, *Transform) {
	return nil, &Transform{m: C.voxel_transform_create_from_frustum((*C.double)((unsafe.Pointer)(&bbox.m[0])), C.double(taper), C.double(depth), C.double(voxelSize))}
}

func (t *Transform) Free() {
	C.voxel_transform_free(t.m)
	t.m = nil
}

func (t *Transform) Clone() *Transform {
	return &Transform{
		m: C.voxel_transform_duplicate(t.m),
	}
}

func (t *Transform) IsLinear() bool {
	return bool(C.voxel_transform_is_linear(t.m))
}

func (t *Transform) IsIdentity() bool {
	return bool(C.voxel_transform_is_identity(t.m))
}

func (t *Transform) HasUniformScale() bool {
	return bool(C.voxel_transform_has_uniform_scale(t.m))
}

func (t *Transform) PreRotate(radians float64, axis Axis) {
	C.voxel_transform_pre_rotate(t.m, C.double(radians), C.int(axis))
}

func (t *Transform) PreTranslate(tran []float64) {
	C.voxel_transform_pre_translate(t.m, (*C.double)((unsafe.Pointer)(&tran[0])))
}

func (t *Transform) PreSaleVector(v []float64) {
	C.voxel_transform_pre_scale_vector(t.m, (*C.double)((unsafe.Pointer)(&v[0])))
}

func (t *Transform) PreSale(s float64) {
	C.voxel_transform_pre_scale(t.m, C.double(s))
}

func (t *Transform) PreShear(shear float64, axis0 Axis, axis1 Axis) {
	C.voxel_transform_pre_shear(t.m, C.double(shear), C.int(axis0), C.int(axis1))
}

func (t *Transform) PreMult(mat []float64) {
	if len(mat) == 9 {
		C.voxel_transform_pre_mult3d(t.m, (*C.double)((unsafe.Pointer)(&mat[0])))
	} else {
		C.voxel_transform_pre_mult4d(t.m, (*C.double)((unsafe.Pointer)(&mat[0])))
	}
}

func (t *Transform) PostRotate(radians float64, axis Axis) {
	C.voxel_transform_post_rotate(t.m, C.double(radians), C.int(axis))
}

func (t *Transform) PostTranslate(tran []float64) {
	C.voxel_transform_post_translate(t.m, (*C.double)((unsafe.Pointer)(&tran[0])))
}

func (t *Transform) PostSaleVector(v []float64) {
	C.voxel_transform_post_scale_vector(t.m, (*C.double)((unsafe.Pointer)(&v[0])))
}

func (t *Transform) PostSale(s float64) {
	C.voxel_transform_post_scale(t.m, C.double(s))
}

func (t *Transform) PostShear(shear float64, axis0 Axis, axis1 Axis) {
	C.voxel_transform_post_shear(t.m, C.double(shear), C.int(axis0), C.int(axis1))
}

func (t *Transform) PostMult(mat []float64) {
	if len(mat) == 9 {
		C.voxel_transform_post_mult3d(t.m, (*C.double)((unsafe.Pointer)(&mat[0])))
	} else {
		C.voxel_transform_post_mult4d(t.m, (*C.double)((unsafe.Pointer)(&mat[0])))
	}
}

func (t *Transform) VoxelSize(xyz []float64) []float64 {
	var si [3]C.double
	if xyz == nil {
		C.voxel_transform_voxel_size(t.m, nil, &si[0])
	} else {
		C.voxel_transform_voxel_size(t.m, (*C.double)((unsafe.Pointer)(&xyz[0])), &si[0])
	}
	return []float64{float64(si[0]), float64(si[1]), float64(si[2])}
}

func (t *Transform) VoxelVolume(xyz []float64) float64 {
	if xyz == nil {
		return float64(C.voxel_transform_voxel_volume(t.m, nil))
	} else {
		return float64(C.voxel_transform_voxel_volume(t.m, (*C.double)((unsafe.Pointer)(&xyz[0]))))
	}
}

func (t *Transform) IndexToWorldFromXYZ(xyz []float64) []float64 {
	var pos [3]C.double
	C.voxel_transform_index_to_world_from_xyz(t.m, (*C.double)((unsafe.Pointer)(&xyz[0])), &pos[0])
	return []float64{float64(pos[0]), float64(pos[1]), float64(pos[2])}
}

func (t *Transform) IndexToWorldFromIJK(ijk []int32) []float64 {
	var pos [3]C.double
	C.voxel_transform_index_to_world_from_ijk(t.m, (*C.int)((unsafe.Pointer)(&ijk[0])), &pos[0])
	return []float64{float64(pos[0]), float64(pos[1]), float64(pos[2])}
}

func (t *Transform) WorldToIndexFromXYZ(xyz []float64) []float64 {
	var indexs [3]C.double
	C.voxel_transform_world_to_index_from_xyz(t.m, (*C.double)((unsafe.Pointer)(&xyz[0])), &indexs[0])
	return []float64{float64(indexs[0]), float64(indexs[1]), float64(indexs[2])}
}

func (t *Transform) WorldToIndexCellCentered(xyz []float64) []int32 {
	var coords [3]C.int
	C.voxel_transform_world_to_index_cell_centered(t.m, (*C.double)((unsafe.Pointer)(&xyz[0])), &coords[0])
	return []int32{int32(coords[0]), int32(coords[1]), int32(coords[2])}
}

func (t *Transform) WorldToIndexNodeCentered(xyz []float64) []int32 {
	var coords [3]C.int
	C.voxel_transform_world_to_index_node_centered(t.m, (*C.double)((unsafe.Pointer)(&xyz[0])), &coords[0])
	return []int32{int32(coords[0]), int32(coords[1]), int32(coords[2])}
}

func (t *Transform) IndexToWorldFromCoordBox(in *CoordBox) *BBox {
	_, pos := NewBBox(make([]float64, 6))
	C.voxel_transform_index_to_world_from_coordbox(t.m, &in.m[0], &pos.m[0])
	return pos
}

func (t *Transform) IndexToWorldFromBBox(in *BBox) *BBox {
	_, pos := NewBBox(make([]float64, 6))
	C.voxel_transform_index_to_world_from_bbox(t.m, &in.m[0], &pos.m[0])
	return pos
}

func (t *Transform) WorldToIndexFromBBox(in *BBox) *BBox {
	_, pos := NewBBox(make([]float64, 6))
	C.voxel_transform_world_to_index_from_bbox(t.m, &in.m[0], &pos.m[0])
	return pos
}

func (t *Transform) WorldToIndexCellCenteredFromBBox(in *BBox) *CoordBox {
	_, pos := NewCoordBox(make([]int32, 6))
	C.voxel_transform_world_to_index_cell_centered_from_bbox(t.m, &in.m[0], &pos.m[0])
	return pos
}

func (t *Transform) WorldToIndexCellNodeCenteredFromBBox(in *BBox) *CoordBox {
	_, pos := NewCoordBox(make([]int32, 6))
	C.voxel_transform_world_to_index_node_centered_from_bbox(t.m, &in.m[0], &pos.m[0])
	return pos
}
