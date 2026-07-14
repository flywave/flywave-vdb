package vdb

// #include <stdlib.h>
// #include "renderer_api.h"
// #cgo CFLAGS: -I ./lib
// #cgo CXXFLAGS: -I ./lib
import "C"
import (
	"errors"
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

func (m *Renderer) RenderScene() error {
	if !bool(C.voxel_renderer_render_scene(m.m)) {
		return errors.New("render scene failed")
	}
	return nil
}

func (m *Renderer) GetFrameBuffer(resolution []int32) ([]uint8, error) {
	var buf *C.uchar
	if !bool(C.voxel_renderer_frame_buffer(m.m, (*C.int)((unsafe.Pointer)(&resolution[0])), &buf)) {
		return nil, errors.New("get frame buffer failed")
	}

	si := uint32(resolution[0] * resolution[1] * 3)

	raw := make([]uint8, si)

	var src []uint8
	aHeader := (*reflect.SliceHeader)((unsafe.Pointer(&src)))
	aHeader.Cap = int(si)
	aHeader.Len = int(si)
	aHeader.Data = uintptr(unsafe.Pointer(buf))

	copy(raw, src)

	return raw, nil
}

func (m *Renderer) GetFrameBufferImage(resolution []int32) (*image.NRGBA, error) {
	pix, err := m.GetFrameBuffer(resolution)
	if err != nil {
		return nil, err
	}
	w := int(resolution[0])
	h := int(resolution[1])
	rect := image.Rectangle{Min: image.Pt(0, 0), Max: image.Pt(w, h)}
	img := image.NewNRGBA(rect)
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			src := (y*w + x) * 3
			dst := y*img.Stride + x*4
			img.Pix[dst+0] = pix[src+0]
			img.Pix[dst+1] = pix[src+1]
			img.Pix[dst+2] = pix[src+2]
			img.Pix[dst+3] = 255
		}
	}
	return img, nil
}

func (m *Renderer) Relocate(path string, copyRelocate bool) error {
	cpath := C.CString(path)
	defer C.free(unsafe.Pointer(cpath))
	if !bool(C.voxel_renderer_relocate(m.m, cpath, C.bool(copyRelocate))) {
		return errors.New("relocate failed")
	}
	return nil
}

func (m *Renderer) ZipArchive(path string, compressionLevel int32) error {
	cpath := C.CString(path)
	defer C.free(unsafe.Pointer(cpath))
	if !bool(C.voxel_renderer_ziparchive(m.m, cpath, C.int(compressionLevel))) {
		return errors.New("zip archive failed")
	}
	return nil
}

func (m *Renderer) Denoiser(path string) error {
	cpath := C.CString(path)
	defer C.free(unsafe.Pointer(cpath))
	if !bool(C.voxel_renderer_denoiser(m.m, cpath)) {
		return errors.New("denoiser failed")
	}
	return nil
}
