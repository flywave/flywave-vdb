package vdb

// #include <stdlib.h>
// #include "vdb_api.h"
// #cgo CFLAGS: -I ./
// #cgo CXXFLAGS: -I -std=gnu++14
import "C"

type Grid struct {
	m *C.struct__vdb_grid_t
}

func NewGrid() *Grid {
	return &Grid{
		m: C.vdb_create()
	}
}

func (m *Grid) Free() {
	C.vdb_free(m.m)
	m.m = nil
}
