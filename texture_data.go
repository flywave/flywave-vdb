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

type PixelFormat uint16

const (
	R    = PixelFormat(0)
	RGB  = PixelFormat(1)
	RGBA = PixelFormat(2)
)

type TextureData struct {
	m *C.struct__voxel_pixel_texture_data_t
}

func NewTextureData(raw []byte, width uint32, height uint32, format PixelFormat) *TextureData {
	var cdata C.struct__c_texture_data_t
	cdata.width = C.size_t(width)
	cdata.height = C.size_t(height)
	cdata.format = C.ushort(format)
	cdata.data = (*C.uchar)((unsafe.Pointer)(&raw[0]))
	return &TextureData{m: C.voxel_pixel_texture_data_create(cdata)}
}

func (t *TextureData) Free() {
	C.voxel_pixel_texture_data_free(t.m)
	t.m = nil
}

func (t *TextureData) Set(raw []byte, width uint32, height uint32, format PixelFormat) {
	var cdata C.struct__c_texture_data_t
	cdata.width = C.size_t(width)
	cdata.height = C.size_t(height)
	cdata.format = C.ushort(format)
	cdata.data = (*C.uchar)((unsafe.Pointer)(&raw[0]))
	C.voxel_pixel_texture_data_set(t.m, cdata)
}

func (t *TextureData) Get() (raw []byte, width uint32, height uint32, format PixelFormat) {
	cdata := C.voxel_pixel_texture_data_get(t.m)

	width = uint32(cdata.width)
	height = uint32(cdata.height)
	format = PixelFormat(cdata.format)

	si := uint32(width * height)
	if format == RGB {
		si *= 3
	} else if format == RGBA {
		si *= 4
	}
	raw = make([]byte, si)

	var src []byte
	aHeader := (*reflect.SliceHeader)((unsafe.Pointer(&src)))
	aHeader.Cap = int(si)
	aHeader.Len = int(si)
	aHeader.Data = uintptr(unsafe.Pointer(cdata.data))

	copy(raw, src)

	return
}
