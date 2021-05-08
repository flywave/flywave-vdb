package vdb

// #include <stdlib.h>
// #include "voxelizer_api.h"
// #cgo CFLAGS: -I ./lib
// #cgo CXXFLAGS: -I ./lib
import "C"
import (
	"unsafe"
)

type BBox struct {
	m [6]C.double
}

type CoordBox struct {
	m [6]C.uint
}