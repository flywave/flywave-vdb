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

type FilterTriangle interface {
	Vaild([]float32, []float32, []float32) bool
}

type FilterTriangleAdapter struct {
	m *C.struct__voxel_filter_triangle_t
	f FilterTriangle
}

func (m *FilterTriangleAdapter) Free() {
	C.voxel_filter_triangle_free(m.m)
	m.m = nil
}

func NewFilterTriangleAdapter(f FilterTriangle) *FilterTriangleAdapter {
	ctx := new(FilterTriangleAdapter)
	ctx.f = f
	inptr := uintptr(unsafe.Pointer(ctx))
	ctx.m = C.voxel_filter_triangle_create(unsafe.Pointer(&inptr))
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

	return C.bool((*(**FilterTriangleAdapter)(ctx)).f.Vaild(aSlice, bSlice, cSlice))
}
