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

type MeshMaterial struct {
	ID        int32
	Faces     []uint32
	Texcoords []uint32
	Normals   []uint32
}

type MeshModel struct {
	Vertices  []float32
	Texcoords []float32
	Normals   []float32
	Materials []MeshMaterial
}

type MeshData struct {
	m *C.struct__voxel_pixel_mesh_data_t
}

func NewMeshData(m *MeshModel) *MeshData {
	var cmeshData C.struct__c_mesh_data_t
	cmeshData.vertices = (*C.float)((unsafe.Pointer)(&m.Vertices[0]))
	cmeshData.v_count = C.size_t(len(m.Vertices) / 3)

	cmeshData.texcoords = (*C.float)((unsafe.Pointer)(&m.Texcoords[0]))
	cmeshData.t_count = C.size_t(len(m.Texcoords) / 2)

	cmeshData.normals = (*C.float)((unsafe.Pointer)(&m.Normals[0]))
	cmeshData.n_count = C.size_t(len(m.Normals) / 3)

	C.voxel_pixel_c_mesh_data_alloc(&cmeshData, C.size_t(len(m.Materials)))

	defer C.voxel_pixel_c_mesh_data_free(&cmeshData)

	var trisSlice []C.struct__c_mesh_data_mtl_t
	trisHeader := (*reflect.SliceHeader)((unsafe.Pointer(&trisSlice)))
	trisHeader.Cap = int(len(m.Materials))
	trisHeader.Len = int(len(m.Materials))
	trisHeader.Data = uintptr(unsafe.Pointer(cmeshData.mtl_map))

	for i := 0; i < len(m.Materials); i++ {
		trisSlice[i].mtl = C.int(m.Materials[i].ID)

		trisSlice[i].faces = (*C.uint)((unsafe.Pointer)(&m.Materials[i].Faces[0]))
		trisSlice[i].f_count = C.size_t(len(m.Materials[i].Faces) / 3)

		trisSlice[i].texcoords = (*C.uint)((unsafe.Pointer)(&m.Materials[i].Texcoords[0]))
		trisSlice[i].t_count = C.size_t(len(m.Materials[i].Texcoords) / 3)

		trisSlice[i].normals = (*C.uint)((unsafe.Pointer)(&m.Materials[i].Normals[0]))
		trisSlice[i].n_count = C.size_t(len(m.Materials[i].Normals) / 3)
	}

	return &MeshData{m: C.voxel_pixel_mesh_data_create(cmeshData)}
}

func (t *MeshData) Free() {
	C.voxel_pixel_mesh_data_free(t.m)
	t.m = nil
}

func (t *MeshData) Get() *MeshModel {
	cmeshData := C.voxel_pixel_mesh_data_get(t.m)
	m := &MeshModel{}
	m.Vertices = make([]float32, int(cmeshData.v_count)*3)
	defer C.voxel_pixel_c_mesh_data_free(&cmeshData)

	var verticesSlice []C.float
	verticesHeader := (*reflect.SliceHeader)((unsafe.Pointer(&verticesSlice)))
	verticesHeader.Cap = int(len(m.Vertices))
	verticesHeader.Len = int(len(m.Vertices))
	verticesHeader.Data = uintptr(unsafe.Pointer(cmeshData.vertices))

	for i := 0; i < len(m.Vertices); i++ {
		m.Vertices[i] = float32(verticesSlice[i])
	}

	m.Texcoords = make([]float32, int(cmeshData.t_count)*2)

	var texcoordsSlice []C.float
	texcoordsHeader := (*reflect.SliceHeader)((unsafe.Pointer(&texcoordsSlice)))
	texcoordsHeader.Cap = int(len(m.Texcoords))
	texcoordsHeader.Len = int(len(m.Texcoords))
	texcoordsHeader.Data = uintptr(unsafe.Pointer(cmeshData.texcoords))

	for i := 0; i < len(m.Texcoords); i++ {
		m.Texcoords[i] = float32(texcoordsSlice[i])
	}

	m.Normals = make([]float32, int(cmeshData.n_count)*3)

	var normalsSlice []C.float
	normalsHeader := (*reflect.SliceHeader)((unsafe.Pointer(&normalsSlice)))
	normalsHeader.Cap = int(len(m.Normals))
	normalsHeader.Len = int(len(m.Normals))
	normalsHeader.Data = uintptr(unsafe.Pointer(cmeshData.texcoords))

	for i := 0; i < len(m.Normals); i++ {
		m.Normals[i] = float32(normalsSlice[i])
	}

	m.Materials = make([]MeshMaterial, int(cmeshData.mtl_count))

	var mtlSlice []C.struct__c_mesh_data_mtl_t
	mtlHeader := (*reflect.SliceHeader)((unsafe.Pointer(&mtlSlice)))
	mtlHeader.Cap = int(len(m.Materials))
	mtlHeader.Len = int(len(m.Materials))
	mtlHeader.Data = uintptr(unsafe.Pointer(cmeshData.mtl_map))

	for i := 0; i < len(m.Materials); i++ {
		m.Materials[i].ID = int32(mtlSlice[i].mtl)

		if int(mtlSlice[i].f_count) > 0 {
			m.Materials[i].Faces = make([]uint32, int(mtlSlice[i].f_count)*3)

			var facesSlice []C.uint
			facesHeader := (*reflect.SliceHeader)((unsafe.Pointer(&facesSlice)))
			facesHeader.Cap = int(mtlSlice[i].f_count * 3)
			facesHeader.Len = int(mtlSlice[i].f_count * 3)
			facesHeader.Data = uintptr(unsafe.Pointer(mtlSlice[i].faces))

			for j := 0; j < int(mtlSlice[i].f_count*3); j++ {
				m.Materials[i].Faces[j] = uint32(facesSlice[j])
			}
		}

		if int(mtlSlice[i].n_count) > 0 {

			m.Materials[i].Normals = make([]uint32, int(mtlSlice[i].n_count)*3)

			var normalsSlice []C.uint
			normalsHeader := (*reflect.SliceHeader)((unsafe.Pointer(&normalsSlice)))
			normalsHeader.Cap = int(mtlSlice[i].n_count * 3)
			normalsHeader.Len = int(mtlSlice[i].n_count * 3)
			normalsHeader.Data = uintptr(unsafe.Pointer(mtlSlice[i].faces))

			for j := 0; j < int(mtlSlice[i].n_count*3); j++ {
				m.Materials[i].Normals[j] = uint32(normalsSlice[j])
			}
		}

		if int(mtlSlice[i].t_count) > 0 {
			m.Materials[i].Texcoords = make([]uint32, int(mtlSlice[i].t_count)*3)

			var texcoordsSlice []C.uint
			texcoordsHeader := (*reflect.SliceHeader)((unsafe.Pointer(&texcoordsSlice)))
			texcoordsHeader.Cap = int(mtlSlice[i].t_count * 3)
			texcoordsHeader.Len = int(mtlSlice[i].t_count * 3)
			texcoordsHeader.Data = uintptr(unsafe.Pointer(mtlSlice[i].faces))

			for j := 0; j < int(mtlSlice[i].t_count*3); j++ {
				m.Materials[i].Texcoords[j] = uint32(texcoordsSlice[j])
			}
		}
	}
	return m
}

func (t *MeshData) Set(m *MeshModel) {
	var cmeshData C.struct__c_mesh_data_t
	cmeshData.vertices = (*C.float)((unsafe.Pointer)(&m.Vertices[0]))
	cmeshData.v_count = C.size_t(len(m.Vertices) / 3)

	cmeshData.texcoords = (*C.float)((unsafe.Pointer)(&m.Texcoords[0]))
	cmeshData.t_count = C.size_t(len(m.Texcoords) / 2)

	cmeshData.normals = (*C.float)((unsafe.Pointer)(&m.Normals[0]))
	cmeshData.n_count = C.size_t(len(m.Normals) / 3)

	C.voxel_pixel_c_mesh_data_alloc(&cmeshData, C.size_t(len(m.Materials)))

	defer C.voxel_pixel_c_mesh_data_free(&cmeshData)

	var trisSlice []C.struct__c_mesh_data_mtl_t
	trisHeader := (*reflect.SliceHeader)((unsafe.Pointer(&trisSlice)))
	trisHeader.Cap = int(len(m.Materials))
	trisHeader.Len = int(len(m.Materials))
	trisHeader.Data = uintptr(unsafe.Pointer(cmeshData.mtl_map))

	for i := 0; i < len(m.Materials); i++ {
		trisSlice[i].mtl = C.int(m.Materials[i].ID)

		trisSlice[i].faces = (*C.uint)((unsafe.Pointer)(&m.Materials[i].Faces[0]))
		trisSlice[i].f_count = C.size_t(len(m.Materials[i].Faces) / 3)

		trisSlice[i].texcoords = (*C.uint)((unsafe.Pointer)(&m.Materials[i].Texcoords[0]))
		trisSlice[i].t_count = C.size_t(len(m.Materials[i].Texcoords) / 3)

		trisSlice[i].normals = (*C.uint)((unsafe.Pointer)(&m.Materials[i].Normals[0]))
		trisSlice[i].n_count = C.size_t(len(m.Materials[i].Normals) / 3)
	}

	C.voxel_pixel_mesh_data_set(t.m, cmeshData)
}

type VoxelMesh struct {
	m *C.struct__voxel_mesh_t
}

func (t *VoxelMesh) Free() {
	C.voxel_mesh_free(t.m)
	t.m = nil
}

func (t *VoxelMesh) SampleVoxelPixel(mtls *Materials, local_feature uint16, precision float32, creator ClipBoxCreateor, _type SamplerType, matrix []float64) *VoxelPixel {
	ccreator := NewClipBoxCreateorAdapter(creator)
	defer ccreator.Free()
	return &VoxelPixel{m: C.voxel_mesh_to_voxel_pixel(t.m, mtls.m, C.ushort(local_feature), C.float(precision), ccreator.m, C.int(_type), (*C.double)((unsafe.Pointer)(&matrix[0])))}
}
