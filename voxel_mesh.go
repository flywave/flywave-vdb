package vdb

// #include <stdlib.h>
// #include "voxelizer_api.h"
// #cgo CFLAGS: -I ./lib
// #cgo CXXFLAGS: -I ./lib
import "C"
import (
	"unsafe"
)

type VoxelMesh struct {
	m *C.struct__voxel_mesh_t
}

func (t *VoxelMesh) Free() {
	C.voxel_mesh_free(t.m)
	t.m = nil
}

func (t *VoxelMesh) Empty() bool {
	return bool(C.voxel_mesh_empty(t.m))
}

func (t *VoxelMesh) Clear() {
	C.voxel_mesh_clear(t.m)
}

func (t *VoxelMesh) SampleVoxelPixel(mtls *Materials, local_feature uint16, precision float32, creator ClipBoxCreateor, _type SamplerType, matrix []float64) *VoxelPixel {
	ccreator := NewClipBoxCreateorAdapter(creator)
	defer ccreator.Free()
	mpt := (*C.struct__voxel_pixel_materials_t)(nil)
	if mtls != nil {
		mpt = mtls.m
	}
	return &VoxelPixel{m: C.voxel_mesh_to_voxel_pixel(t.m, mpt, C.ushort(local_feature), C.float(precision), ccreator.m, C.int(_type), (*C.double)((unsafe.Pointer)(&matrix[0])))}
}
