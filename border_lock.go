package vdb

// #include <stdlib.h>
// #include "voxelizer_api.h"
// #cgo CFLAGS: -I ./lib
// #cgo CXXFLAGS: -I ./lib
import "C"
import (
	"unsafe"
)

//export borderLockCheck
func borderLockCheck(ctx unsafe.Pointer, a *C.float) C.bool {

}

type BorderLock struct {
}
