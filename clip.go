package vdb

// #include <stdlib.h>
// #include "voxelizer_api.h"
// #cgo CFLAGS: -I ./lib
// #cgo CXXFLAGS: -I ./lib
import "C"
import (
	"unsafe"
)

//export clipBoxCreateor
func clipBoxCreateor(ctx unsafe.Pointer, vertex *C.struct__vdb_float_grid_t, tran *C.struct__voxel_transform_t, sbox *C.double, cbox *C.double) C.bool {
	return false
}

type ClipBoxCreateor struct {
	m *C.struct__voxel_clip_box_createor_t
}
