package vdb

// #include <stdlib.h>
// #include "voxelizer_api.h"
// #cgo CFLAGS: -I ./lib
// #cgo CXXFLAGS: -I ./lib
import "C"
import (
	"reflect"
	"unsafe"

	"github.com/flywave/go3d/vec3"
)

type BorderLock interface {
	Check(*vec3.T) bool
}

type BorderLockAdapter struct {
	m *C.struct__voxel_border_lock_t
	f BorderLock
}

func (m *BorderLockAdapter) Free() {
	C.voxel_border_lock_free(m.m)
	m.m = nil
}

func NewBorderLockAdapter(f BorderLock) *BorderLockAdapter {
	ctx := new(BorderLockAdapter)
	ctx.f = f
	inptr := uintptr(unsafe.Pointer(ctx))
	ctx.m = C.voxel_border_lock_create(unsafe.Pointer(&inptr))
	return ctx
}

//export borderLockCheck
func borderLockCheck(ctx unsafe.Pointer, a *C.float) C.bool {
	var cpointsSlice []float32
	cpointsHeader := (*reflect.SliceHeader)((unsafe.Pointer(&cpointsSlice)))
	cpointsHeader.Cap = int(3)
	cpointsHeader.Len = int(3)
	cpointsHeader.Data = uintptr(unsafe.Pointer(a))

	var vec vec3.T
	copy(vec[:], cpointsSlice)
	return C.bool((*(**BorderLockAdapter)(ctx)).f.Check(&vec))
}
