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

type Texture2D struct {
	m *C.struct__voxel_texture2d_t
}

func NewTexture2D(width uint32, height uint32) *Texture2D {
	return &Texture2D{m: C.voxel_texture2d_create(C.uint(width), C.uint(height))}
}

func (t *Texture2D) Free() {
	C.voxel_texture2d_free(t.m)
	t.m = nil
}

func (t *Texture2D) GetRawData() ([]byte, uint32, uint32) {
	var data *C.uchar
	var width C.uint
	var height C.uint
	C.voxel_texture2d_get_raw_data(t.m, &data, &width, &height)
	defer C.free(unsafe.Pointer(data))

	si := uint32(width) * uint32(height) * uint32(4)

	var dataSlice []C.uchar
	dataHeader := (*reflect.SliceHeader)((unsafe.Pointer(&dataSlice)))
	dataHeader.Cap = int(si)
	dataHeader.Len = int(si)
	dataHeader.Data = uintptr(unsafe.Pointer(data))

	ret := make([]byte, int(si))

	for i := 0; i < int(si); i++ {
		ret[i] = byte(dataSlice[i])
	}
	return ret, uint32(width), uint32(height)
}

func (t *Texture2D) SetRawData(raw []byte, width uint32, height uint32) {
	C.voxel_texture2d_set_raw_data(t.m, (*C.uchar)((unsafe.Pointer)(&raw[0])), C.uint(width), C.uint(height))
}

func (m *Texture2D) Clone() *Texture2D {
	return &Texture2D{
		m: C.voxel_texture2d_duplicate(m.m),
	}
}

func (t *Texture2D) FillPixel(data byte) {
	C.voxel_texture2d_fill_pixel(t.m, C.uchar(data))
}

func (t *Texture2D) FillColor(raw []byte) {
	C.voxel_texture2d_fill_color(t.m, (*C.uchar)((unsafe.Pointer)(&raw[0])))
}

func (t *Texture2D) Resize(width uint32, height uint32) {
	C.voxel_texture2d_resize(t.m, C.uint(width), C.uint(height))
}
