package vdb

import (
	"testing"

	"github.com/flywave/go3d/vec3"
)

func TestMeshModelAppendTriangle(t *testing.T) {
	m := &MeshModel{}
	m.appendTriangle(1, 2, 3)
	if len(m.Materials) != 1 {
		t.Fatalf("expected 1 material, got %d", len(m.Materials))
	}
	mtl := m.Materials[0]
	if mtl.ID != -1 {
		t.Errorf("default material ID = %d, want -1", mtl.ID)
	}
	if len(mtl.Faces) != 3 || mtl.Faces[0] != 1 || mtl.Faces[2] != 3 {
		t.Errorf("Faces = %v, want [1 2 3]", mtl.Faces)
	}
	// appendTriangle mirrors indices into Texcoords and Normals.
	if len(mtl.Texcoords) != 3 || len(mtl.Normals) != 3 {
		t.Errorf("Texcoords/Normals not populated")
	}
	// A second append reuses the same (last) material.
	m.appendTriangle(4, 5, 6)
	if len(m.Materials) != 1 {
		t.Errorf("expected still 1 material, got %d", len(m.Materials))
	}
	if len(m.Materials[0].Faces) != 6 {
		t.Errorf("Faces len = %d, want 6", len(m.Materials[0].Faces))
	}
}

func TestMeshModelBuildBox(t *testing.T) {
	m := &MeshModel{}
	m.buildBox(2, 2)
	// A box has 6 quads -> 12 triangles -> 36 indices.
	const faces = 12 * 3
	total := 0
	for _, mtl := range m.Materials {
		total += len(mtl.Faces)
	}
	if total != faces {
		t.Errorf("total face indices = %d, want %d", total, faces)
	}
	// 6 quads * 4 vertices = 24 vertices -> 72 floats.
	if len(m.Vertices) != 24*3 {
		t.Errorf("vertices floats = %d, want %d", len(m.Vertices), 24*3)
	}
	if len(m.UVs) != 24*2 {
		t.Errorf("uv floats = %d, want %d", len(m.UVs), 24*2)
	}
	if len(m.Normals) != 24*3 {
		t.Errorf("normal floats = %d, want %d", len(m.Normals), 24*3)
	}
	// All face indices must be within the vertex count (24 verts).
	vertCount := uint32(len(m.Vertices) / 3)
	for _, mtl := range m.Materials {
		for _, idx := range mtl.Faces {
			if idx >= vertCount {
				t.Errorf("face index %d out of range [0,%d)", idx, vertCount)
			}
		}
	}
}

func TestToCFloatPtrNilOnEmpty(t *testing.T) {
	if got := toCFloatPtr(nil); got != nil {
		t.Errorf("toCFloatPtr(nil) != nil")
	}
	if got := toCFloatPtr([]float32{}); got != nil {
		t.Errorf("toCFloatPtr(empty) != nil")
	}
	if got := toCUintPtr(nil); got != nil {
		t.Errorf("toCUintPtr(nil) != nil")
	}
	if got := toCUintPtr([]uint32{}); got != nil {
		t.Errorf("toCUintPtr(empty) != nil")
	}
}

func TestBuildBoxGeometrySanity(t *testing.T) {
	// A unit-ish box centered near origin: vertex extents should be bounded by
	// the requested length/width in each axis.
	m := &MeshModel{}
	m.buildBox(2, 2)
	min, max := vec3.T{1e9, 1e9, 1e9}, vec3.T{-1e9, -1e9, -1e9}
	for i := 0; i < len(m.Vertices); i += 3 {
		v := vec3.T{m.Vertices[i], m.Vertices[i+1], m.Vertices[i+2]}
		for j := 0; j < 3; j++ {
			if v[j] < min[j] {
				min[j] = v[j]
			}
			if v[j] > max[j] {
				max[j] = v[j]
			}
		}
	}
	// Box spans -2..0 in each axis (length=width=2, nearCorner=origin).
	if min[0] != -2 || max[0] != 0 {
		t.Errorf("x extent [%f,%f], want [-2,0]", min[0], max[0])
	}
}
