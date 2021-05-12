package vdb

// #include <stdlib.h>
// #include "renderer_api.h"
// #cgo CFLAGS: -I ./lib
// #cgo CXXFLAGS: -I ./lib
import "C"
import (
	"image"
	"reflect"
	"unsafe"
)

type Renderer struct {
	m *C.struct__voxel_renderer_t
}

func NewRenderer() *Renderer {
	return &Renderer{
		m: C.voxel_renderer_create(),
	}
}

func (m *Renderer) Free() {
	C.voxel_renderer_free(m.m)
	m.m = nil
}

func (m *Renderer) SetCheckpointInterva(val float64) {
	C.voxel_renderer_set_checkpoint_interval(m.m, C.double(val))
}

func (m *Renderer) SetTimeout(t float64) {
	C.voxel_renderer_set_timeout(m.m, C.double(t))
}

func (m *Renderer) SetThreadCount(c int32) {
	C.voxel_renderer_set_thread_count(m.m, C.int(c))
}

func (m *Renderer) SetInputDirectory(path string) {
	cpath := C.CString(path)
	C.voxel_renderer_set_input_directory(m.m, cpath)
	C.free(unsafe.Pointer(cpath))
}

func (m *Renderer) SetOutputDirectory(path string) {
	cpath := C.CString(path)
	C.voxel_renderer_set_output_directory(m.m, cpath)
	C.free(unsafe.Pointer(cpath))
}

func (m *Renderer) Setup() {
	C.voxel_renderer_setup(m.m)
}

func (m *Renderer) RenderScene() {
	C.voxel_renderer_render_scene(m.m)
}

func (m *Renderer) GetFrameBuffer(resolution []int32) []uint8 {
	var buf *C.uchar
	C.voxel_renderer_frame_buffer(m.m, (*C.int)((unsafe.Pointer)(&resolution[0])), &buf)

	si := uint32(resolution[0] * resolution[1] * 3)

	raw := make([]uint8, si)

	var src []uint8
	aHeader := (*reflect.SliceHeader)((unsafe.Pointer(&src)))
	aHeader.Cap = int(si)
	aHeader.Len = int(si)
	aHeader.Data = uintptr(unsafe.Pointer(buf))

	copy(raw, src)

	return raw
}

func (m *Renderer) GetFrameBufferImage(resolution []int32) *image.NRGBA {
	pix := m.GetFrameBuffer(resolution)
	rect := image.Rectangle{Min: image.Pt(0, 0), Max: image.Pt(int(resolution[0]), int(resolution[1]))}

	return &image.NRGBA{
		Pix:    pix,
		Stride: 4 * rect.Dx(),
		Rect:   rect,
	}
}

func (m *Renderer) Relocate(path string, copyRelocate bool) {
	cpath := C.CString(path)
	C.voxel_renderer_relocate(m.m, cpath, C.bool(copyRelocate))
	C.free(unsafe.Pointer(cpath))
}

func (m *Renderer) ZipArchive(path string, compressionLevel int32) {
	cpath := C.CString(path)
	C.voxel_renderer_ziparchive(m.m, cpath, C.int(compressionLevel))
	C.free(unsafe.Pointer(cpath))
}

func (m *Renderer) Denoiser(path string) {
	cpath := C.CString(path)
	C.voxel_renderer_denoiser(m.m, cpath)
	C.free(unsafe.Pointer(cpath))
}
