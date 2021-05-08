package vdb

// #include <stdlib.h>
// #include "voxelizer_api.h"
// #cgo CFLAGS: -I ./lib
// #cgo CXXFLAGS: -I ./lib
import "C"
import "errors"

type BBox struct {
	m []C.double
}

func NewBBox(bbox []float64) (error, *BBox) {
	if len(bbox) != 6 {
		return errors.New("bbox size must 6"), nil
	}
	return nil, &BBox{m: []C.double{C.double(bbox[0]), C.double(bbox[1]), C.double(bbox[2]), C.double(bbox[3]), C.double(bbox[4]), C.double(bbox[5])}}
}

func (b *BBox) Set(bbox []float64) {
	for i := 0; i <= 6; i++ {
		b.m[i] = C.double(bbox[i])
	}
}

func (b *BBox) SetMin(min []float64) {
	for i := 0; i <= 3; i++ {
		b.m[i] = C.double(min[i])
	}
}

func (b *BBox) SetMax(max []float64) {
	for i := 0; i <= 3; i++ {
		b.m[i+3] = C.double(max[i])
	}
}

func (b *BBox) GetMin() []float64 {
	return []float64{float64(b.m[0]), float64(b.m[1]), float64(b.m[2])}
}

func (b *BBox) GetMax() []float64 {
	return []float64{float64(b.m[3]), float64(b.m[4]), float64(b.m[5])}
}

type CoordBox struct {
	m []C.int
}

func NewCoordBox(cbox []int32) (error, *CoordBox) {
	if len(cbox) != 6 {
		return errors.New("cbox size must 6"), nil
	}
	return nil, &CoordBox{m: []C.int{C.int(cbox[0]), C.int(cbox[1]), C.int(cbox[2]), C.int(cbox[3]), C.int(cbox[4]), C.int(cbox[5])}}
}

func (b *CoordBox) Set(cbox []int32) {
	for i := 0; i <= 6; i++ {
		b.m[i] = C.int(cbox[i])
	}
}

func (b *CoordBox) SetMin(min []int32) {
	for i := 0; i <= 3; i++ {
		b.m[i] = C.int(min[i])
	}
}

func (b *CoordBox) SetMax(max []int32) {
	for i := 0; i <= 3; i++ {
		b.m[i+3] = C.int(max[i])
	}
}

func (b *CoordBox) GetMin() []int32 {
	return []int32{int32(b.m[0]), int32(b.m[1]), int32(b.m[2])}
}

func (b *CoordBox) GetMax() []int32 {
	return []int32{int32(b.m[3]), int32(b.m[4]), int32(b.m[5])}
}
