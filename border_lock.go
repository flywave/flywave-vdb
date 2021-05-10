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

type BorderLock struct {
	m     *C.struct__voxel_border_lock_t
	check func([]float32) bool
}

func (m *BorderLock) Free() {
	C.voxel_border_lock_free(m.m)
	m.m = nil
}

func NewBorderLock(f func([]float32) bool) *BorderLock {
	ctx := &BorderLock{check: f}
	ctx.m = C.voxel_border_lock_create(unsafe.Pointer(ctx))
	return ctx
}

//export borderLockCheck
func borderLockCheck(ctx unsafe.Pointer, a *C.float) C.bool {
	var cpointsSlice []float32
	cpointsHeader := (*reflect.SliceHeader)((unsafe.Pointer(&cpointsSlice)))
	cpointsHeader.Cap = int(3)
	cpointsHeader.Len = int(3)
	cpointsHeader.Data = uintptr(unsafe.Pointer(a))

	return C.bool((*BorderLock)(ctx).check(cpointsSlice))
}
