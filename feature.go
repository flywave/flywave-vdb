package vdb

// #include <stdlib.h>
// #include "voxelizer_api.h"
// #cgo CFLAGS: -I ./lib
// #cgo CXXFLAGS: -I ./lib
import "C"
import (
	"reflect"
	"unsafe"
)

type Features struct {
	m *C.struct__voxel_pixel_features_t
}

func (t *Features) Free() {
	C.voxel_pixel_features_free(t.m)
	t.m = nil
}

type FeatureModel struct {
	LocalID  uint8
	GlobalID uint64
	Data     []byte
}

type FeatureData struct {
	m *C.struct__voxel_pixel_feature_data_t
}

func NewFeatureData(model FeatureModel) *FeatureData {
	var cdata C.struct__c_feature_data_t
	cdata.local = C.uchar(model.LocalID)
	cdata.global = C.ulong(model.GlobalID)
	cdata.size = C.size_t(len(model.Data))
	cdata.data = (*C.uchar)((unsafe.Pointer)(&model.Data[0]))
	return &FeatureData{m: C.voxel_pixel_feature_data_create(cdata)}
}

func (t *FeatureData) Free() {
	C.voxel_pixel_feature_data_free(t.m)
	t.m = nil
}

func (t *FeatureData) Set(model FeatureModel) {
	var cdata C.struct__c_feature_data_t
	cdata.local = C.uchar(model.LocalID)
	cdata.global = C.ulong(model.GlobalID)
	cdata.size = C.size_t(len(model.Data))
	cdata.data = (*C.uchar)((unsafe.Pointer)(&model.Data[0]))
	C.voxel_pixel_feature_data_set(t.m, cdata)
}

func (t *FeatureData) Get() *FeatureModel {
	cdata := C.voxel_pixel_feature_data_get(t.m)

	local := uint8(cdata.local)
	global := uint64(cdata.global)

	si := uint32(cdata.size)
	raw := make([]byte, si)

	var src []byte
	aHeader := (*reflect.SliceHeader)((unsafe.Pointer(&src)))
	aHeader.Cap = int(si)
	aHeader.Len = int(si)
	aHeader.Data = uintptr(unsafe.Pointer(cdata.data))

	copy(raw, src)

	return &FeatureModel{LocalID: local, GlobalID: global, Data: raw}
}
