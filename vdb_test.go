package vdb

import "testing"

func TestBBox(t *testing.T) {
	raw := []float64{2, 2, 2, 4, 4, 4}
	_, bbox := NewBBox(raw)

	print(bbox.GetMin())
}

func TestRegex(t *testing.T) {
	zxystr := _parsr_file_reg.FindStringSubmatch("1_2_3.vtile")

	print(len(zxystr))
}
