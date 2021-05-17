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

type LocalFeatureID uint16
type FeatureID uint64

type Features struct {
	m *C.struct__voxel_pixel_features_t
}

func (t *Features) Free() {
	C.voxel_pixel_features_free(t.m)
	t.m = nil
}

func (t *Features) Count() int {
	return int(C.voxel_pixel_features_get_features_count(t.m))
}

func (t *Features) Append(model FeatureModel) {
	data := NewFeatureData(model)
	defer data.Free()
	C.voxel_pixel_features_append_feature(t.m, data.m)
}

func (t *Features) Set(i int, model FeatureModel) {
	data := NewFeatureData(model)
	defer data.Free()
	C.voxel_pixel_features_set_feature(t.m, C.int(int32(i)), data.m)
}

func (t *Features) Get(i int) *FeatureModel {
	data := FeatureData{m: C.voxel_pixel_features_get_feature(t.m, C.int(int32(i)))}
	defer data.Free()
	return data.Get()
}

func (t *Features) Slice() []FeatureModel {
	models := make([]FeatureModel, t.Count())
	for i := 0; i < len(models); i++ {
		models[i] = *t.Get(i)
	}
	return models
}

type FeatureModel struct {
	GlobalID FeatureID
	Data     []byte
}

type FeatureData struct {
	m *C.struct__voxel_pixel_feature_data_t
}

func NewFeatureData(model FeatureModel) *FeatureData {
	var cdata C.struct__c_feature_data_t
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
	cdata.global = C.ulong(model.GlobalID)
	cdata.size = C.size_t(len(model.Data))
	cdata.data = (*C.uchar)((unsafe.Pointer)(&model.Data[0]))

	C.voxel_pixel_feature_data_set(t.m, cdata)
}

func (t *FeatureData) Get() *FeatureModel {
	cdata := C.voxel_pixel_feature_data_get(t.m)

	global := FeatureID(cdata.global)

	si := uint32(cdata.size)
	raw := make([]byte, si)

	var src []byte
	aHeader := (*reflect.SliceHeader)((unsafe.Pointer(&src)))
	aHeader.Cap = int(si)
	aHeader.Len = int(si)
	aHeader.Data = uintptr(unsafe.Pointer(cdata.data))

	copy(raw, src)

	return &FeatureModel{GlobalID: global, Data: raw}
}
