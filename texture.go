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

func (t *Texture2D) GetRawData() (ret []uint8, width uint32, height uint32) {
	var data *C.uchar
	var cwidth C.uint
	var cheight C.uint
	C.voxel_texture2d_get_raw_data(t.m, &data, &cwidth, &cheight)
	defer C.free(unsafe.Pointer(data))

	si := uint32(cwidth) * uint32(cheight) * uint32(4)

	var dataSlice []C.uchar
	dataHeader := (*reflect.SliceHeader)((unsafe.Pointer(&dataSlice)))
	dataHeader.Cap = int(si)
	dataHeader.Len = int(si)
	dataHeader.Data = uintptr(unsafe.Pointer(data))

	ret = make([]uint8, int(si))

	for i := 0; i < int(si); i++ {
		ret[i] = uint8(dataSlice[i])
	}
	width = uint32(cwidth)
	height = uint32(cheight)

	return
}

func (t *Texture2D) SetRawData(raw []uint8, width uint32, height uint32) {
	C.voxel_texture2d_set_raw_data(t.m, (*C.uchar)((unsafe.Pointer)(&raw[0])), C.uint(width), C.uint(height))
}

func (m *Texture2D) Clone() *Texture2D {
	return &Texture2D{
		m: C.voxel_texture2d_duplicate(m.m),
	}
}

func (t *Texture2D) FillPixel(data uint8) {
	C.voxel_texture2d_fill_pixel(t.m, C.uchar(data))
}

func (t *Texture2D) FillColor(c color.NRGBA) {
	raw := []uint8{c.R, c.G, c.B, c.A}
	C.voxel_texture2d_fill_color(t.m, (*C.uchar)((unsafe.Pointer)(&raw[0])))
}

func (t *Texture2D) Resize(width uint32, height uint32) {
	C.voxel_texture2d_resize(t.m, C.uint(width), C.uint(height))
}

func (t *Texture2D) SetImage(img *image.NRGBA) {
	t.SetRawData(img.Pix, uint32(img.Rect.Dx()), uint32(img.Rect.Dy()))
}

func (m *Texture2D) GetImage() *image.NRGBA {
	pix, w, h := m.GetRawData()
	rect := image.Rectangle{Min: image.Pt(0, 0), Max: image.Pt(int(w), int(h))}

	return &image.NRGBA{
		Pix:    pix,
		Stride: 4 * rect.Dx(),
		Rect:   rect,
	}
}
