package vdb

import "testing"

func TestBBox(t *testing.T) {
	raw := []float64{2, 2, 2, 4, 4, 4}
	_, bbox := NewBBox(raw)

	print(bbox.GetMin())
}

func TestCoordBox(t *testing.T) {
	raw := []int32{2, 2, 2, 4, 4, 4}
	_, cbox := NewCoordBox(raw)

	print(cbox.GetMin())
}
