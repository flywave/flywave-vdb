package vdb

// #include <stdlib.h>
// #include "voxelizer_api.h"
// #cgo CFLAGS: -I ./lib
// #cgo CXXFLAGS: -I ./lib
import "C"
import (
	"errors"
	"math"
)

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

type BBox2d [4]float64

func NewBBox2d() *BBox2d {
	return &BBox2d{-math.MaxFloat64, -math.MaxFloat64, math.MaxFloat64, math.MaxFloat64}
}

func (b BBox2d) Width() float64 {
	return b[2] - b[0]
}

func (b BBox2d) Height() float64 {
	return b[3] - b[1]
}

func (b BBox2d) Size() float64 {
	width := b.Width()
	height := b.Height()
	return math.Max(width, height)
}

func (b *BBox2d) add(p []float64) {
	(*b)[0] = math.Min((*b)[0], p[0])
	(*b)[1] = math.Min((*b)[1], p[1])
	(*b)[2] = math.Max((*b)[2], p[0])
	(*b)[3] = math.Max((*b)[3], p[1])
}

func (b *BBox2d) Add(p interface{}) {
	switch t := p.(type) {
	case []int:
	case []float32:
		b.add([]float64{float64(t[0]), float64(t[1])})
	case []float64:
		b.add(t)
	case [3]float64:
		b.add(t[:])
	case [2]float64:
		b.add(t[:])
	}
}

func (b *BBox2d) Grow(delta float64) {
	(*b)[0] -= delta
	(*b)[1] -= delta
	(*b)[2] += delta
	(*b)[3] += delta
}

func epseq(l, r, epsilon float64) bool {
	return math.Abs(l-r) < epsilon
}

func (b BBox2d) IsOnBorder(point []float64, epsilon float64) bool {
	return epseq(point[0], b[0], epsilon) || epseq(point[0], b[2], epsilon) ||
		epseq(point[1], b[1], epsilon) || epseq(point[1], b[3], epsilon)
}

// https://stackoverflow.com/questions/306316/determine-if-two-rectangles-overlap-each-other
func (b BBox2d) Intersects(o BBox2d, epsilon float64) bool {
	if b[1]-epsilon > o[3]+epsilon {
		return false
	}

	if b[3]+epsilon < o[1]-epsilon {
		return false
	}

	if b[2]+epsilon < o[0]-epsilon {
		return false
	}

	if b[0]-epsilon > o[2]+epsilon {
		return false
	}

	return true
}

func (b BBox2d) Contains(point []float64, epsilon float64) bool {
	return (b[0]-epsilon) <= point[0] && (b[1]-epsilon) <= point[1] &&
		(b[2]+epsilon) >= point[0] && (b[3]+epsilon) >= point[1]
}
