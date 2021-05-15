package vdb

// #include <stdlib.h>
// #include "voxelizer_api.h"
// #cgo CFLAGS: -I ./lib
// #cgo CXXFLAGS: -I ./lib
import "C"
import (
	"math"

	vec2d "github.com/flywave/go3d/float64/vec2"
)

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

func (b *BBox2d) add(p vec2d.T) {
	(*b)[0] = math.Min((*b)[0], p[0])
	(*b)[1] = math.Min((*b)[1], p[1])
	(*b)[2] = math.Max((*b)[2], p[0])
	(*b)[3] = math.Max((*b)[3], p[1])
}

func (b *BBox2d) Add(p interface{}) {
	switch t := p.(type) {
	case []int:
	case []float32:
		b.add(vec2d.T{float64(t[0]), float64(t[1])})
	case []float64:
		b.add(vec2d.T{t[0], t[1]})
	case [3]float64:
		b.add(vec2d.T{t[0], t[1]})
	case [2]float64:
		b.add(vec2d.T{t[0], t[1]})
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

func (b BBox2d) IsOnBorder(point vec2d.T, epsilon float64) bool {
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

func (b BBox2d) Contains(point vec2d.T, epsilon float64) bool {
	return (b[0]-epsilon) <= point[0] && (b[1]-epsilon) <= point[1] &&
		(b[2]+epsilon) >= point[0] && (b[3]+epsilon) >= point[1]
}

func (b BBox2d) ContainsBox(box BBox2d, epsilon float64) bool {
	return (box[0]-epsilon) >= b[0] && (box[2]+epsilon) <= b[2] && (box[1]-epsilon) >= b[1] && (box[3]+epsilon) <= b[3]
}
