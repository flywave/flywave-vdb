package vdb

// #include <stdlib.h>
// #include "voxelizer_api.h"
// #cgo CFLAGS: -I ./lib
// #cgo CXXFLAGS: -I ./lib
import "C"
import "unsafe"

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
