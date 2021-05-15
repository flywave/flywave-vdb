package vdb

// #include <stdlib.h>
// #include "voxelizer_api.h"
// #cgo CFLAGS: -I ./lib
// #cgo CXXFLAGS: -I ./lib
import "C"
import (
	"unsafe"

	"github.com/flywave/flywave-vdb/coord"
	mat3d "github.com/flywave/go3d/float64/mat3"
	mat4d "github.com/flywave/go3d/float64/mat4"
	vec3d "github.com/flywave/go3d/float64/vec3"
)

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

func NewTransformFromMat4d(mat mat4d.T) (error, *Transform) {
	return nil, &Transform{m: C.voxel_transform_create_from_mat((*C.double)((unsafe.Pointer)(&mat.Slice()[0])))}
}

func NewTransformFromFrustum(bbox *vec3d.Box, taper float64, depth float64, voxelSize float64) (error, *Transform) {
	return nil, &Transform{m: C.voxel_transform_create_from_frustum((*C.double)((unsafe.Pointer)(&bbox.Slice()[0])), C.double(taper), C.double(depth), C.double(voxelSize))}
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

func (t *Transform) PreTranslate(tran vec3d.T) {
	C.voxel_transform_pre_translate(t.m, (*C.double)((unsafe.Pointer)(&tran[0])))
}

func (t *Transform) PreSaleVector(v vec3d.T) {
	C.voxel_transform_pre_scale_vector(t.m, (*C.double)((unsafe.Pointer)(&v[0])))
}

func (t *Transform) PreSale(s float64) {
	C.voxel_transform_pre_scale(t.m, C.double(s))
}

func (t *Transform) PreShear(shear float64, axis0 Axis, axis1 Axis) {
	C.voxel_transform_pre_shear(t.m, C.double(shear), C.int(axis0), C.int(axis1))
}

func (t *Transform) PreMult3d(mat mat3d.T) {
	C.voxel_transform_pre_mult3d(t.m, (*C.double)((unsafe.Pointer)(&mat[0])))
}

func (t *Transform) PreMult4d(mat mat4d.T) {
	C.voxel_transform_pre_mult4d(t.m, (*C.double)((unsafe.Pointer)(&mat[0])))
}

func (t *Transform) PostRotate(radians float64, axis Axis) {
	C.voxel_transform_post_rotate(t.m, C.double(radians), C.int(axis))
}

func (t *Transform) PostTranslate(tran vec3d.T) {
	C.voxel_transform_post_translate(t.m, (*C.double)((unsafe.Pointer)(&tran[0])))
}

func (t *Transform) PostSaleVector(v vec3d.T) {
	C.voxel_transform_post_scale_vector(t.m, (*C.double)((unsafe.Pointer)(&v[0])))
}

func (t *Transform) PostSale(s float64) {
	C.voxel_transform_post_scale(t.m, C.double(s))
}

func (t *Transform) PostShear(shear float64, axis0 Axis, axis1 Axis) {
	C.voxel_transform_post_shear(t.m, C.double(shear), C.int(axis0), C.int(axis1))
}

func (t *Transform) PostMult3d(mat mat3d.T) {
	C.voxel_transform_post_mult3d(t.m, (*C.double)((unsafe.Pointer)(&mat.Slice()[0])))
}

func (t *Transform) PostMult4d(mat mat3d.T) {
	C.voxel_transform_post_mult4d(t.m, (*C.double)((unsafe.Pointer)(&mat.Slice()[0])))
}

func (t *Transform) VoxelSize(xyz *vec3d.T) vec3d.T {
	var si [3]C.double
	if xyz == nil {
		C.voxel_transform_voxel_size(t.m, nil, &si[0])
	} else {
		C.voxel_transform_voxel_size(t.m, (*C.double)((unsafe.Pointer)(&xyz[0])), &si[0])
	}
	return vec3d.T{float64(si[0]), float64(si[1]), float64(si[2])}
}

func (t *Transform) VoxelVolume(xyz *vec3d.T) float64 {
	if xyz == nil {
		return float64(C.voxel_transform_voxel_volume(t.m, nil))
	} else {
		return float64(C.voxel_transform_voxel_volume(t.m, (*C.double)((unsafe.Pointer)(&xyz[0]))))
	}
}

func (t *Transform) IndexToWorldFromXYZ(xyz vec3d.T) vec3d.T {
	var pos [3]C.double
	C.voxel_transform_index_to_world_from_xyz(t.m, (*C.double)((unsafe.Pointer)(&xyz[0])), &pos[0])
	return vec3d.T{float64(pos[0]), float64(pos[1]), float64(pos[2])}
}

func (t *Transform) IndexToWorldFromIJK(ijk coord.T) vec3d.T {
	var pos [3]C.double
	C.voxel_transform_index_to_world_from_ijk(t.m, (*C.int)((unsafe.Pointer)(&ijk[0])), &pos[0])
	return vec3d.T{float64(pos[0]), float64(pos[1]), float64(pos[2])}
}

func (t *Transform) WorldToIndexFromXYZ(xyz vec3d.T) vec3d.T {
	var indexs [3]C.double
	C.voxel_transform_world_to_index_from_xyz(t.m, (*C.double)((unsafe.Pointer)(&xyz[0])), &indexs[0])
	return vec3d.T{float64(indexs[0]), float64(indexs[1]), float64(indexs[2])}
}

func (t *Transform) WorldToIndexCellCentered(xyz vec3d.T) coord.T {
	var coords [3]C.int
	C.voxel_transform_world_to_index_cell_centered(t.m, (*C.double)((unsafe.Pointer)(&xyz[0])), &coords[0])
	return coord.T{int32(coords[0]), int32(coords[1]), int32(coords[2])}
}

func (t *Transform) WorldToIndexNodeCentered(xyz vec3d.T) coord.T {
	var coords [3]C.int
	C.voxel_transform_world_to_index_node_centered(t.m, (*C.double)((unsafe.Pointer)(&xyz[0])), &coords[0])
	return coord.T{int32(coords[0]), int32(coords[1]), int32(coords[2])}
}

func (t *Transform) IndexToWorldFromCoordBox(in *coord.Box) *vec3d.Box {
	pos := make([]float64, 6)
	C.voxel_transform_index_to_world_from_coordbox(t.m, (*C.int)(in.CSlice()), (*C.double)((unsafe.Pointer)(&pos[0])))
	return vec3d.FromSlice(pos)
}

func (t *Transform) IndexToWorldFromBBox(in *vec3d.Box) *vec3d.Box {
	pos := make([]float64, 6)
	C.voxel_transform_index_to_world_from_bbox(t.m, (*C.double)((unsafe.Pointer)(&in.Slice()[0])), (*C.double)((unsafe.Pointer)(&pos[0])))
	return vec3d.FromSlice(pos)
}

func (t *Transform) WorldToIndexFromBBox(in *vec3d.Box) *vec3d.Box {
	pos := make([]float64, 6)
	C.voxel_transform_world_to_index_from_bbox(t.m, (*C.double)((unsafe.Pointer)(&in.Slice()[0])), (*C.double)((unsafe.Pointer)(&pos[0])))
	return vec3d.FromSlice(pos)
}

func (t *Transform) WorldToIndexCellCenteredFromBBox(in *vec3d.Box) *coord.Box {
	cb := make([]int32, 6)
	C.voxel_transform_world_to_index_cell_centered_from_bbox(t.m, (*C.double)((unsafe.Pointer)(&in.Slice()[0])), (*C.int)((unsafe.Pointer)(&cb[0])))
	return coord.FromSlice(cb)
}

func (t *Transform) WorldToIndexCellNodeCenteredFromBBox(in *vec3d.Box) *coord.Box {
	cb := make([]int32, 6)
	C.voxel_transform_world_to_index_node_centered_from_bbox(t.m, (*C.double)((unsafe.Pointer)(&in.Slice()[0])), (*C.int)((unsafe.Pointer)(&cb[0])))
	return coord.FromSlice(cb)
}
