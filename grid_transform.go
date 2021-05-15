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

type GridTransform interface {
	IsAffine() bool
	IsIdentity() bool
	Transform(pt, out *vec3d.T)
	InvTransform(pt, out *vec3d.T)
}

type GridTransformAdapter struct {
	m *C.struct__voxel_grid_transform_t
	f GridTransform
}

func NewGridTransformAdapter(f GridTransform) *GridTransformAdapter {
	ctx := new(GridTransformAdapter)
	ctx.f = f
	inptr := uintptr(unsafe.Pointer(ctx))
	ctx.m = C.voxel_grid_transform_create(unsafe.Pointer(&inptr))
	return ctx
}

func (m *GridTransformAdapter) Free() {
	C.voxel_grid_transform_free(m.m)
	m.m = nil
}

//export isAffine
func isAffine(ctx unsafe.Pointer) C.bool {
	return C.bool((*(**GridTransformAdapter)(ctx)).f.IsAffine())
}

//export isIdentity
func isIdentity(ctx unsafe.Pointer) C.bool {
	return C.bool((*(**GridTransformAdapter)(ctx)).f.IsIdentity())
}

//export gridTransform
func gridTransform(ctx unsafe.Pointer, pt *C.double, out *C.double) {
	var aSlice []float64
	aHeader := (*reflect.SliceHeader)((unsafe.Pointer(&aSlice)))
	aHeader.Cap = int(3)
	aHeader.Len = int(3)
	aHeader.Data = uintptr(unsafe.Pointer(pt))

	var bSlice []float64
	bHeader := (*reflect.SliceHeader)((unsafe.Pointer(&bSlice)))
	bHeader.Cap = int(3)
	bHeader.Len = int(3)
	bHeader.Data = uintptr(unsafe.Pointer(out))

	var avec vec3d.T
	copy(avec[:], aSlice)

	var bvec vec3d.T
	copy(bvec[:], bSlice)

	(*(**GridTransformAdapter)(ctx)).f.Transform(&avec, &bvec)
}

//export gridInvTransform
func gridInvTransform(ctx unsafe.Pointer, pt *C.double, out *C.double) {
	var aSlice []float64
	aHeader := (*reflect.SliceHeader)((unsafe.Pointer(&aSlice)))
	aHeader.Cap = int(3)
	aHeader.Len = int(3)
	aHeader.Data = uintptr(unsafe.Pointer(pt))

	var bSlice []float64
	bHeader := (*reflect.SliceHeader)((unsafe.Pointer(&bSlice)))
	bHeader.Cap = int(3)
	bHeader.Len = int(3)
	bHeader.Data = uintptr(unsafe.Pointer(out))

	var avec vec3d.T
	copy(avec[:], aSlice)

	var bvec vec3d.T
	copy(bvec[:], bSlice)

	(*(**GridTransformAdapter)(ctx)).f.InvTransform(&avec, &bvec)
}
