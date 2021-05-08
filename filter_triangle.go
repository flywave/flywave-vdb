package vdb

// #include <stdlib.h>
// #include "voxelizer_api.h"
// #cgo CFLAGS: -I ./lib
// #cgo CXXFLAGS: -I ./lib
import "C"
import (
	"unsafe"
)

//export filterTriangleValid
func filterTriangleValid(ctx unsafe.Pointer, a *C.float, b *C.float, c *C.float) C.bool {

}

type FilterTriangle struct {
}
