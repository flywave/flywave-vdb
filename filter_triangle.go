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

type FilterTriangle struct {
	m     *C.struct__voxel_filter_triangle_t
	vaild func([]float32, []float32, []float32) bool
}

func (m *FilterTriangle) Free() {
	C.voxel_filter_triangle_free(m.m)
	m.m = nil
}

func NewFilterTriangle(f func([]float32, []float32, []float32) bool) *FilterTriangle {
	ctx := &FilterTriangle{vaild: f}
	ctx.m = C.voxel_filter_triangle_create(unsafe.Pointer(ctx))
	return ctx
}

//export filterTriangleValid
func filterTriangleValid(ctx unsafe.Pointer, a *C.float, b *C.float, c *C.float) C.bool {
	var aSlice []float32
	aHeader := (*reflect.SliceHeader)((unsafe.Pointer(&aSlice)))
	aHeader.Cap = int(3)
	aHeader.Len = int(3)
	aHeader.Data = uintptr(unsafe.Pointer(a))

	var bSlice []float32
	bHeader := (*reflect.SliceHeader)((unsafe.Pointer(&bSlice)))
	bHeader.Cap = int(3)
	bHeader.Len = int(3)
	bHeader.Data = uintptr(unsafe.Pointer(b))

	var cSlice []float32
	cHeader := (*reflect.SliceHeader)((unsafe.Pointer(&cSlice)))
	cHeader.Cap = int(3)
	cHeader.Len = int(3)
	cHeader.Data = uintptr(unsafe.Pointer(c))

	return C.bool((*FilterTriangle)(ctx).vaild(aSlice, bSlice, cSlice))
}
