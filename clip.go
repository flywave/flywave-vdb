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

type ClipBoxCreateor struct {
	m   *C.struct__voxel_clip_box_createor_t
	gen func(*FloatGrid, *Transform, *BBox, *BBox) bool
}

func NewClipBoxCreateor(f func(*FloatGrid, *Transform, *BBox, *BBox) bool) *ClipBoxCreateor {
	ctx := &ClipBoxCreateor{gen: f}
	ctx.m = C.voxel_clip_box_createor_create((unsafe.Pointer)(ctx))
	return ctx
}

func (m *ClipBoxCreateor) Free() {
	C.voxel_clip_box_createor_free(m.m)
	m.m = nil
}

//export clipBoxCreateor
func clipBoxCreateor(ctx unsafe.Pointer, vertex *C.struct__vdb_float_grid_t, tran *C.struct__voxel_transform_t, sbox *C.double, cbox *C.double) C.bool {
	grid := &FloatGrid{m: vertex}
	defer grid.Free()

	t := &Transform{m: tran}
	defer t.Free()

	var sboxSlice []float64
	sboxHeader := (*reflect.SliceHeader)((unsafe.Pointer(&sboxSlice)))
	sboxHeader.Cap = int(3)
	sboxHeader.Len = int(3)
	sboxHeader.Data = uintptr(unsafe.Pointer(sbox))

	var cboxSlice []C.double
	cboxHeader := (*reflect.SliceHeader)((unsafe.Pointer(&cboxSlice)))
	cboxHeader.Cap = int(3)
	cboxHeader.Len = int(3)
	cboxHeader.Data = uintptr(unsafe.Pointer(cbox))

	outbox := &BBox{}
	_, inbox := NewBBox(sboxSlice)

	ret := C.bool((*ClipBoxCreateor)(ctx).gen(grid, t, inbox, outbox))

	for i := 0; i <= 6; i++ {
		cboxSlice[i] = outbox.m[i]
	}
	return ret
}
