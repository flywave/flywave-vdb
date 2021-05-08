package vdb

// #include <stdlib.h>
// #include "voxelizer_api.h"
// #cgo CFLAGS: -I ./lib
// #cgo CXXFLAGS: -I ./lib
import "C"
import (
	"reflect"
	"unsafe"
)

type AtlasGenerator struct {
	m *C.struct__voxel_texture_atlas_generator_t
}

func (m *AtlasGenerator) Free() {
	C.voxel_texture_atlas_generator_free(m.m)
	m.m = nil
}

func (m *AtlasGenerator) Generate(mesh *TextureMesh) []Texture2D {
	var tex **C.struct__voxel_texture2d_t
	var si C.size_t

	C.voxel_texture_atlas_generator_generate(m.m, mesh.m, &tex, &si)

	defer C.free(unsafe.Pointer(tex))

	var texSlice []*C.struct__voxel_texture2d_t
	texHeader := (*reflect.SliceHeader)((unsafe.Pointer(&texSlice)))
	texHeader.Cap = int(si)
	texHeader.Len = int(si)
	texHeader.Data = uintptr(unsafe.Pointer(tex))

	ret := make([]Texture2D, int(si))

	for i := 0; i < int(si); i++ {
		ret[i].m = texSlice[i]
	}

	return ret
}
