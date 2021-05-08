package vdb

// #include <stdlib.h>
// #include "voxelizer_api.h"
// #cgo CFLAGS: -I ./lib
// #cgo CXXFLAGS: -I ./lib
import "C"

type Materials struct {
	m *C.struct__voxel_pixel_materials_t
}

func (t *Materials) Free() {
	C.voxel_pixel_materials_free(t.m)
	t.m = nil
}

type MaterialModel struct {
	ID                 uint8
	TP                 uint32
	Mode               uint16
	Color              [3]uint8
	Ambient            [3]uint8
	Emissive           [3]uint8
	Specular           [3]uint8
	Opacity            float32
	Shininess          float32
	Metallic           float32
	Roughness          float32
	Reflectance        float32
	ClearcoatThickness float32
	ClearcoatRoughness float32
	Anisotropy         float32
	AnisotropyRotation float32
}

type MaterialData struct {
	m *C.struct__voxel_pixel_material_data_t
}

func NewMaterialData(m MaterialModel) *MaterialData {
	var cmdata C.struct__c_material_data_t
	cmdata._material_id = C.uchar(m.ID)
	cmdata._type = C.uint(m.TP)
	cmdata.mode = C.ushort(m.Mode)
	cmdata.color_r = C.uchar(m.Color[0])
	cmdata.color_g = C.uchar(m.Color[1])
	cmdata.color_b = C.uchar(m.Color[2])
	cmdata.ambient_r = C.uchar(m.Ambient[0])
	cmdata.ambient_g = C.uchar(m.Ambient[1])
	cmdata.ambient_b = C.uchar(m.Ambient[2])
	cmdata.emissive_r = C.uchar(m.Emissive[0])
	cmdata.emissive_g = C.uchar(m.Emissive[1])
	cmdata.emissive_b = C.uchar(m.Emissive[2])
	cmdata.specular_r = C.uchar(m.Specular[0])
	cmdata.specular_g = C.uchar(m.Specular[1])
	cmdata.specular_b = C.uchar(m.Specular[2])
	cmdata.opacity = C.float(m.Opacity)
	cmdata.shininess = C.float(m.Shininess)
	cmdata.metallic = C.float(m.Metallic)
	cmdata.roughness = C.float(m.Roughness)
	cmdata.reflectance = C.float(m.Reflectance)
	cmdata.clearcoat_thickness = C.float(m.ClearcoatThickness)
	cmdata.clearcoat_roughness = C.float(m.ClearcoatRoughness)
	cmdata.anisotropy = C.float(m.Anisotropy)
	cmdata.anisotropy_rotation = C.float(m.AnisotropyRotation)

	return &MaterialData{m: C.voxel_pixel_material_data_create(cmdata)}
}

func (t *MaterialData) Free() {
	C.voxel_pixel_material_data_free(t.m)
	t.m = nil
}

func (t *MaterialData) Set(m MaterialModel) {
	var cmdata C.struct__c_material_data_t
	cmdata._material_id = C.uchar(m.ID)
	cmdata._type = C.uint(m.TP)
	cmdata.mode = C.ushort(m.Mode)
	cmdata.color_r = C.uchar(m.Color[0])
	cmdata.color_g = C.uchar(m.Color[1])
	cmdata.color_b = C.uchar(m.Color[2])
	cmdata.ambient_r = C.uchar(m.Ambient[0])
	cmdata.ambient_g = C.uchar(m.Ambient[1])
	cmdata.ambient_b = C.uchar(m.Ambient[2])
	cmdata.emissive_r = C.uchar(m.Emissive[0])
	cmdata.emissive_g = C.uchar(m.Emissive[1])
	cmdata.emissive_b = C.uchar(m.Emissive[2])
	cmdata.specular_r = C.uchar(m.Specular[0])
	cmdata.specular_g = C.uchar(m.Specular[1])
	cmdata.specular_b = C.uchar(m.Specular[2])
	cmdata.opacity = C.float(m.Opacity)
	cmdata.shininess = C.float(m.Shininess)
	cmdata.metallic = C.float(m.Metallic)
	cmdata.roughness = C.float(m.Roughness)
	cmdata.reflectance = C.float(m.Reflectance)
	cmdata.clearcoat_thickness = C.float(m.ClearcoatThickness)
	cmdata.clearcoat_roughness = C.float(m.ClearcoatRoughness)
	cmdata.anisotropy = C.float(m.Anisotropy)
	cmdata.anisotropy_rotation = C.float(m.AnisotropyRotation)

	C.voxel_pixel_material_data_set(t.m, cmdata)
}

func (t *MaterialData) Get() *MaterialModel {
	cmdata := C.voxel_pixel_material_data_get(t.m)
	return &MaterialModel{ID: uint8(cmdata._material_id), TP: uint32(cmdata._type), Mode: uint16(cmdata.mode), Color: [3]uint8{uint8(cmdata.color_r), uint8(cmdata.color_g), uint8(cmdata.color_b)}, Ambient: [3]uint8{uint8(cmdata.ambient_r), uint8(cmdata.ambient_g), uint8(cmdata.ambient_b)}, Emissive: [3]uint8{uint8(cmdata.emissive_r), uint8(cmdata.emissive_g), uint8(cmdata.emissive_b)}, Specular: [3]uint8{uint8(cmdata.specular_r), uint8(cmdata.specular_g), uint8(cmdata.specular_b)}, Opacity: float32(cmdata.opacity), Shininess: float32(cmdata.shininess), Metallic: float32(cmdata.metallic), Roughness: float32(cmdata.roughness), Reflectance: float32(cmdata.reflectance), ClearcoatThickness: float32(cmdata.clearcoat_thickness), ClearcoatRoughness: float32(cmdata.clearcoat_roughness), Anisotropy: float32(cmdata.anisotropy), AnisotropyRotation: float32(cmdata.anisotropy_rotation)}
}
