package vdb

// #include <stdlib.h>
// #include "voxelizer_api.h"
// #cgo CFLAGS: -I ./lib
// #cgo CXXFLAGS: -I ./lib
import "C"
import "unsafe"

type VoxelMeshBuilder struct {
	m *C.struct__voxel_mesh_builder_t
}

func NewVoxelMeshBuilder() *VoxelMeshBuilder {
	return &VoxelMeshBuilder{m: C.voxel_mesh_builder_create()}
}

func (t *VoxelMeshBuilder) Free() {
	C.voxel_mesh_builder_free(t.m)
	t.m = nil
}

func (t *VoxelMeshBuilder) SetName(name string) {
	cname := C.CString(name)
	C.voxel_mesh_builder_set_name(t.m, cname)
	C.free(unsafe.Pointer(cname))
}

func (t *VoxelMeshBuilder) GetName() string {
	return C.GoString(C.voxel_mesh_builder_get_name(t.m))
}

func (t *VoxelMeshBuilder) AddMesh(m *MeshModel) {
	mdata := NewMeshData(m)
	defer mdata.Free()
	t.addMeshData(mdata)
}

func (t *VoxelMeshBuilder) addMeshData(m *MeshData) {
	C.voxel_mesh_builder_add_mesh_data(t.m, m.m)
}

func (t *VoxelMeshBuilder) AddMaterial(m MaterialModel, index int32) {
	mdata := NewMaterialData(m)
	defer mdata.Free()
	t.addMaterialData(mdata, index)
}

func (t *VoxelMeshBuilder) addMaterialData(m *MaterialData, index int32) {
	C.voxel_mesh_builder_add_material_data(t.m, m.m, C.int(index))
}

func (t *VoxelMeshBuilder) AddTexture(img interface{}, name string) {
	tdata := NewTextureWithImage(img)
	defer tdata.Free()
	t.addTextureData(tdata, name)
}

func (t *VoxelMeshBuilder) addTextureData(m *TextureData, name string) {
	cname := C.CString(name)
	C.voxel_mesh_builder_add_texture_data(t.m, m.m, cname)
	C.free(unsafe.Pointer(cname))
}

func (t *VoxelMeshBuilder) TextureExist(name string) bool {
	cname := C.CString(name)
	C.free(unsafe.Pointer(cname))
	return bool(C.voxel_mesh_builder_texture_exist(t.m, cname))
}

func (t *VoxelMeshBuilder) MaterialExist(index int32) bool {
	return bool(C.voxel_mesh_builder_material_exist(t.m, C.int(index)))
}

func (t *VoxelMeshBuilder) BuildMesh() *VoxelMesh {
	return &VoxelMesh{m: C.voxel_mesh_builder_build_mesh(t.m)}
}

func (t *VoxelMeshBuilder) BuildTextureMesh() *TextureMesh {
	return &TextureMesh{m: C.voxel_mesh_builder_build_texture_mesh(t.m)}
}
