package vdb

// #include <stdlib.h>
// #include "voxelizer_api.h"
// #cgo CFLAGS: -I ./lib
// #cgo CXXFLAGS: -I ./lib
import "C"
import (
	"math"
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
	UVs       []float32
	Normals   []float32
	Materials []MeshMaterial
}

func crossAndNormalized(v1 []float32, v2 []float32) []float32 {
	ret := []float32{0, 0, 0}
	ret[0] += v1[1]*v2[2] + v1[2]*v2[1]
	ret[1] += v1[2]*v2[0] + v1[0]*v2[2]
	ret[2] += v1[0]*v2[1] + v1[1]*v2[0]
	d := math.Sqrt(float64(ret[0]*ret[0] + ret[1]*ret[1] + ret[2]*ret[2]))
	ret[0] *= float32(1 / d)
	ret[1] *= float32(1 / d)
	ret[2] *= float32(1 / d)
	return ret
}

func addVec3(a []float32, b ...[]float32) []float32 {
	ret := []float32{0, 0, 0}
	for i := range b {
		ret[0] += b[i][0]
		ret[1] += b[i][1]
		ret[2] += b[i][2]
	}
	return ret
}

func (m *MeshModel) appendTriangle(a uint32, b uint32, c uint32) {
	if len(m.Materials) == 0 {
		mtl := MeshMaterial{}
		mtl.ID = -1
		m.Materials = append(m.Materials, mtl)
	}
	mtl := &m.Materials[len(m.Materials)-1]
	mtl.Faces = append(mtl.Faces, a, b, c)
	mtl.Texcoords = append(mtl.Texcoords, a, b, c)
	mtl.Normals = append(mtl.Normals, a, b, c)
}

func (m *MeshModel) buildQuad(offset []float32, widthDir []float32, lengthDir []float32) {
	normal := crossAndNormalized(lengthDir, widthDir)

	m.Vertices = append(m.Vertices, offset...)
	m.UVs = append(m.UVs, 0.0, 0.0)
	m.Normals = append(m.Normals, normal...)

	m.Vertices = append(m.Vertices, addVec3(offset, lengthDir)...)
	m.UVs = append(m.UVs, 0.0, 1.0)
	m.Normals = append(m.Normals, normal...)

	m.Vertices = append(m.Vertices, addVec3(offset, lengthDir, widthDir)...)
	m.UVs = append(m.UVs, 1.0, 1.0)
	m.Normals = append(m.Normals, normal...)

	m.Vertices = append(m.Vertices, addVec3(offset, widthDir)...)
	m.UVs = append(m.UVs, 1.0, 0.0)
	m.Normals = append(m.Normals, normal...)

	baseIndex := uint32(len(m.Vertices)/3 - 4)

	m.appendTriangle(baseIndex, baseIndex+1, baseIndex+2)
	m.appendTriangle(baseIndex, baseIndex+2, baseIndex+3)
}

func (m *MeshModel) buildBox(length float32, width float32) {
	upDir := []float32{0, length, 0}
	rightDir := []float32{width, 0, 0}
	forwardDir := []float32{0, 0, length}

	iupDir := []float32{0, -length, 0}
	irightDir := []float32{-width, 0, 0}
	iforwardDir := []float32{0, 0, -length}

	nearCorner := []float32{0, 0, 0}
	farCorner := addVec3(upDir, rightDir, forwardDir)

	m.buildQuad(nearCorner, forwardDir, rightDir)
	m.buildQuad(nearCorner, rightDir, upDir)
	m.buildQuad(nearCorner, upDir, forwardDir)

	m.buildQuad(farCorner, irightDir, iforwardDir)
	m.buildQuad(farCorner, iupDir, irightDir)
	m.buildQuad(farCorner, iforwardDir, iupDir)
}

type MeshData struct {
	m *C.struct__voxel_pixel_mesh_data_t
}

func NewMeshData(m *MeshModel) *MeshData {
	var cmeshData C.struct__c_mesh_data_t
	cmeshData.vertices = (*C.float)((unsafe.Pointer)(&m.Vertices[0]))
	cmeshData.v_count = C.size_t(len(m.Vertices) / 3)

	cmeshData.texcoords = (*C.float)((unsafe.Pointer)(&m.UVs[0]))
	cmeshData.t_count = C.size_t(len(m.UVs) / 2)

	cmeshData.normals = (*C.float)((unsafe.Pointer)(&m.Normals[0]))
	cmeshData.n_count = C.size_t(len(m.Normals) / 3)

	cmeshData.mtl_map = (*C.struct__c_mesh_data_mtl_t)(C.malloc(C.sizeof_struct__c_mesh_data_mtl_t * C.ulong(len(m.Materials))))
	cmeshData.mtl_count = C.size_t(len(m.Materials))

	defer C.free(unsafe.Pointer(cmeshData.mtl_map))

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

	defer C.free(unsafe.Pointer(cmeshData.mtl_map))

	var verticesSlice []C.float
	verticesHeader := (*reflect.SliceHeader)((unsafe.Pointer(&verticesSlice)))
	verticesHeader.Cap = int(len(m.Vertices))
	verticesHeader.Len = int(len(m.Vertices))
	verticesHeader.Data = uintptr(unsafe.Pointer(cmeshData.vertices))

	for i := 0; i < len(m.Vertices); i++ {
		m.Vertices[i] = float32(verticesSlice[i])
	}

	m.UVs = make([]float32, int(cmeshData.t_count)*2)

	var texcoordsSlice []C.float
	texcoordsHeader := (*reflect.SliceHeader)((unsafe.Pointer(&texcoordsSlice)))
	texcoordsHeader.Cap = int(len(m.UVs))
	texcoordsHeader.Len = int(len(m.UVs))
	texcoordsHeader.Data = uintptr(unsafe.Pointer(cmeshData.texcoords))

	for i := 0; i < len(m.UVs); i++ {
		m.UVs[i] = float32(texcoordsSlice[i])
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

	cmeshData.texcoords = (*C.float)((unsafe.Pointer)(&m.UVs[0]))
	cmeshData.t_count = C.size_t(len(m.UVs) / 2)

	cmeshData.normals = (*C.float)((unsafe.Pointer)(&m.Normals[0]))
	cmeshData.n_count = C.size_t(len(m.Normals) / 3)

	cmeshData.mtl_map = (*C.struct__c_mesh_data_mtl_t)(C.malloc(C.sizeof_struct__c_mesh_data_mtl_t * C.ulong(len(m.Materials))))
	cmeshData.mtl_count = C.size_t(len(m.Materials))

	defer C.free(unsafe.Pointer(cmeshData.mtl_map))

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
