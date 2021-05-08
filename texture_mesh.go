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

type TextureMesh struct {
	m *C.struct__voxel_texture_mesh_t
}

func NewTextureMesh() *TextureMesh {
	return &TextureMesh{
		m: C.voxel_texture_mesh_create(),
	}
}

func NewTextureMeshFromTriangles(tris []Triangle) *TextureMesh {
	trisSlice := make([]C.struct__voxel_io_triangle_t, len(tris))

	for i := 0; i < len(tris); i++ {
		trisSlice[i].v_a.v_x = C.float(tris[i].V[0].V[0])
		trisSlice[i].v_a.v_y = C.float(tris[i].V[0].V[1])
		trisSlice[i].v_a.v_z = C.float(tris[i].V[0].V[2])

		trisSlice[i].v_a.c_r = C.uchar(tris[i].V[0].C[0])
		trisSlice[i].v_a.c_g = C.uchar(tris[i].V[0].C[1])
		trisSlice[i].v_a.c_b = C.uchar(tris[i].V[0].C[2])
		trisSlice[i].v_a.c_a = C.uchar(tris[i].V[0].C[3])

		trisSlice[i].v_a.t_u = C.float(tris[i].V[0].T[0])
		trisSlice[i].v_a.t_v = C.float(tris[i].V[0].T[1])

		trisSlice[i].v_a.b = C.bool(tris[i].V[0].B)

		trisSlice[i].v_b.v_x = C.float(tris[i].V[1].V[0])
		trisSlice[i].v_b.v_y = C.float(tris[i].V[1].V[1])
		trisSlice[i].v_b.v_z = C.float(tris[i].V[1].V[2])

		trisSlice[i].v_b.c_r = C.uchar(tris[i].V[1].C[0])
		trisSlice[i].v_b.c_g = C.uchar(tris[i].V[1].C[1])
		trisSlice[i].v_b.c_b = C.uchar(tris[i].V[1].C[2])
		trisSlice[i].v_b.c_a = C.uchar(tris[i].V[1].C[3])

		trisSlice[i].v_b.t_u = C.float(tris[i].V[1].T[0])
		trisSlice[i].v_b.t_v = C.float(tris[i].V[1].T[1])

		trisSlice[i].v_b.b = C.bool(tris[i].V[1].B)

		trisSlice[i].v_c.v_x = C.float(tris[i].V[2].V[0])
		trisSlice[i].v_c.v_y = C.float(tris[i].V[2].V[1])
		trisSlice[i].v_c.v_z = C.float(tris[i].V[2].V[2])

		trisSlice[i].v_c.c_r = C.uchar(tris[i].V[2].C[0])
		trisSlice[i].v_c.c_g = C.uchar(tris[i].V[2].C[1])
		trisSlice[i].v_c.c_b = C.uchar(tris[i].V[2].C[2])
		trisSlice[i].v_c.c_a = C.uchar(tris[i].V[2].C[3])

		trisSlice[i].v_c.t_u = C.float(tris[i].V[2].T[0])
		trisSlice[i].v_c.t_v = C.float(tris[i].V[2].T[1])

		trisSlice[i].v_c.b = C.bool(tris[i].V[2].B)

		trisSlice[i].node = C.uint(tris[i].Node)
		trisSlice[i].tex = C.uint(tris[i].Tex)
		trisSlice[i].mtl = C.uint(tris[i].Mtl)
		trisSlice[i].feature_id = C.ulong(tris[i].FID)
	}

	return &TextureMesh{
		m: C.voxel_texture_mesh_create_from_triangles(&trisSlice[0], C.int(len(tris))),
	}
}

func (t *TextureMesh) Free() {
	C.voxel_texture_mesh_free(t.m)
	t.m = nil
}

func (m *TextureMesh) Clone() *TextureMesh {
	return &TextureMesh{
		m: C.voxel_texture_mesh_duplicate(m.m),
	}
}

func (t *TextureMesh) GetTriangles(node int32) []Triangle {
	var ctris *C.struct__voxel_io_triangle_t
	var csi C.int

	C.voxel_texture_mesh_get_triangles(t.m, &ctris, &csi, C.int(node))

	var trisSlice []C.struct__voxel_io_triangle_t
	trisHeader := (*reflect.SliceHeader)((unsafe.Pointer(&trisSlice)))
	trisHeader.Cap = int(csi)
	trisHeader.Len = int(csi)
	trisHeader.Data = uintptr(unsafe.Pointer(ctris))

	ret := make([]Triangle, int(csi))

	for i := 0; i < int(csi); i++ {
		ret[i].V[0].V[0] = float32(trisSlice[i].v_a.v_x)
		ret[i].V[0].V[1] = float32(trisSlice[i].v_a.v_y)
		ret[i].V[0].V[2] = float32(trisSlice[i].v_a.v_z)

		ret[i].V[0].C[0] = uint8(trisSlice[i].v_a.c_r)
		ret[i].V[0].C[1] = uint8(trisSlice[i].v_a.c_g)
		ret[i].V[0].C[2] = uint8(trisSlice[i].v_a.c_b)
		ret[i].V[0].C[3] = uint8(trisSlice[i].v_a.c_a)

		ret[i].V[0].T[0] = float32(trisSlice[i].v_a.t_u)
		ret[i].V[0].T[1] = float32(trisSlice[i].v_a.t_v)

		ret[i].V[0].B = bool(trisSlice[i].v_a.b)

		ret[i].V[1].V[0] = float32(trisSlice[i].v_b.v_x)
		ret[i].V[1].V[1] = float32(trisSlice[i].v_b.v_y)
		ret[i].V[1].V[2] = float32(trisSlice[i].v_b.v_z)

		ret[i].V[1].C[0] = uint8(trisSlice[i].v_b.c_r)
		ret[i].V[1].C[1] = uint8(trisSlice[i].v_b.c_g)
		ret[i].V[1].C[2] = uint8(trisSlice[i].v_b.c_b)
		ret[i].V[1].C[3] = uint8(trisSlice[i].v_b.c_a)

		ret[i].V[1].T[0] = float32(trisSlice[i].v_b.t_u)
		ret[i].V[1].T[1] = float32(trisSlice[i].v_b.t_v)

		ret[i].V[1].B = bool(trisSlice[i].v_b.b)

		ret[i].V[2].V[0] = float32(trisSlice[i].v_c.v_x)
		ret[i].V[2].V[1] = float32(trisSlice[i].v_c.v_y)
		ret[i].V[2].V[2] = float32(trisSlice[i].v_c.v_z)

		ret[i].V[2].C[0] = uint8(trisSlice[i].v_c.c_r)
		ret[i].V[2].C[1] = uint8(trisSlice[i].v_c.c_g)
		ret[i].V[2].C[2] = uint8(trisSlice[i].v_c.c_b)
		ret[i].V[2].C[3] = uint8(trisSlice[i].v_c.c_a)

		ret[i].V[2].T[0] = float32(trisSlice[i].v_c.t_u)
		ret[i].V[2].T[1] = float32(trisSlice[i].v_c.t_v)

		ret[i].V[2].B = bool(trisSlice[i].v_c.b)

		ret[i].Node = uint32(trisSlice[i].node)
		ret[i].Tex = uint32(trisSlice[i].tex)
		ret[i].Mtl = uint32(trisSlice[i].mtl)
		ret[i].FID = uint64(trisSlice[i].feature_id)
	}

	return ret
}
