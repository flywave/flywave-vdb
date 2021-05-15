package vdb

// #include <stdlib.h>
// #include "voxelizer_api.h"
// #cgo CFLAGS: -I ./lib
// #cgo CXXFLAGS: -I ./lib
import "C"
import (
	"reflect"
	"unsafe"

	vec3d "github.com/flywave/go3d/float64/vec3"
)

type ClipBoxCreateor interface {
	Gen(*FloatGrid, *Transform, *vec3d.Box, *vec3d.Box) bool
}

type NoneClipBoxCreateor struct {
	ClipBoxCreateor
}

func (t *NoneClipBoxCreateor) Gen(vertex *FloatGrid, resolution *Transform, sbox *vec3d.Box, box *vec3d.Box) bool {
	*box = *sbox
	return true
}

type ClipBoxCreateorAdapter struct {
	m *C.struct__voxel_clip_box_createor_t
	f ClipBoxCreateor
}

func NewClipBoxCreateorAdapter(f ClipBoxCreateor) *ClipBoxCreateorAdapter {
	ctx := new(ClipBoxCreateorAdapter)
	ctx.f = f
	inptr := uintptr(unsafe.Pointer(ctx))
	ctx.m = C.voxel_clip_box_createor_create(unsafe.Pointer(&inptr))
	return ctx
}

func (m *ClipBoxCreateorAdapter) Free() {
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
	sboxHeader.Cap = int(6)
	sboxHeader.Len = int(6)
	sboxHeader.Data = uintptr(unsafe.Pointer(sbox))

	var cboxSlice []C.double
	cboxHeader := (*reflect.SliceHeader)((unsafe.Pointer(&cboxSlice)))
	cboxHeader.Cap = int(6)
	cboxHeader.Len = int(6)
	cboxHeader.Data = uintptr(unsafe.Pointer(cbox))

	outbox := new(vec3d.Box)
	inbox := vec3d.FromSlice(sboxSlice)

	ret := C.bool((*(**ClipBoxCreateorAdapter)(ctx)).f.Gen(grid, t, inbox, outbox))

	outSlice := outbox.Slice()
	for i := 0; i < 6; i++ {
		cboxSlice[i] = C.double(outSlice[i])
	}
	return ret
}
