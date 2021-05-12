package vdb

// #include <stdlib.h>
// #include "voxelizer_api.h"
// #cgo CFLAGS: -I ./lib
// #cgo CXXFLAGS: -I ./lib
import "C"
import (
	"image"
	"image/color"
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

func NewTextureData(raw []uint8, width uint32, height uint32, format PixelFormat) *TextureData {
	var cdata C.struct__c_texture_data_t
	cdata.width = C.size_t(width)
	cdata.height = C.size_t(height)
	cdata.format = C.ushort(format)
	cdata.data = (*C.uchar)((unsafe.Pointer)(&raw[0]))
	return &TextureData{m: C.voxel_pixel_texture_data_create(cdata)}
}

func NewTextureWithImage(t interface{}) *TextureData {
	switch img := t.(type) {
	case *image.NRGBA:
		return NewTextureData(img.Pix, uint32(img.Rect.Dx()), uint32(img.Rect.Dy()), RGBA)
	case *image.RGBA:
		return NewTextureData(img.Pix, uint32(img.Rect.Dx()), uint32(img.Rect.Dy()), RGBA)
	case *image.Gray:
		return NewTextureData(img.Pix, uint32(img.Rect.Dx()), uint32(img.Rect.Dy()), R)
	}
	return nil
}

func (t *TextureData) Free() {
	C.voxel_pixel_texture_data_free(t.m)
	t.m = nil
}

func (t *TextureData) SetImage(i interface{}) {
	switch img := i.(type) {
	case *image.NRGBA:
		t.Set(img.Pix, uint32(img.Rect.Dx()), uint32(img.Rect.Dy()), RGBA)
	case *image.RGBA:
		t.Set(img.Pix, uint32(img.Rect.Dx()), uint32(img.Rect.Dy()), RGBA)
	case *image.Gray:
		t.Set(img.Pix, uint32(img.Rect.Dx()), uint32(img.Rect.Dy()), R)
	}
}

func (t *TextureData) Set(raw []uint8, width uint32, height uint32, format PixelFormat) {
	var cdata C.struct__c_texture_data_t
	cdata.width = C.size_t(width)
	cdata.height = C.size_t(height)
	cdata.format = C.ushort(format)
	cdata.data = (*C.uchar)((unsafe.Pointer)(&raw[0]))
	C.voxel_pixel_texture_data_set(t.m, cdata)
}

func (t *TextureData) GetImage() interface{} {
	raw, w, h, fmt := t.Get()
	rect := image.Rectangle{Min: image.Pt(0, 0), Max: image.Pt(int(w), int(h))}

	switch fmt {
	case R:
		return &image.Gray{Pix: raw, Stride: rect.Dx(),
			Rect: rect}
	case RGB:
		rimg := image.NewNRGBA(rect)
		for x := rect.Min.X; x < rect.Max.X; x++ {
			for y := rect.Min.Y; y < rect.Max.Y; y++ {
				rimg.Set(x, y, color.NRGBA{raw[(x*3*int(w))+y], raw[(x*3*int(w))+y+1], raw[(x*3*int(w))+y+2], 255})
			}
		}
	case RGBA:
		return &image.NRGBA{Pix: raw, Stride: 4 * rect.Dx(),
			Rect: rect}
	}
	return nil
}

func (t *TextureData) Get() (raw []uint8, width uint32, height uint32, format PixelFormat) {
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
	raw = make([]uint8, si)

	var src []uint8
	aHeader := (*reflect.SliceHeader)((unsafe.Pointer(&src)))
	aHeader.Cap = int(si)
	aHeader.Len = int(si)
	aHeader.Data = uintptr(unsafe.Pointer(cdata.data))

	copy(raw, src)

	return
}
