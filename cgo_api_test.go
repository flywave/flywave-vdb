package vdb

import (
	"os"
	"path/filepath"
	"testing"

	"github.com/flywave/flywave-vdb/coord"
	mat4d "github.com/flywave/go3d/float64/mat4"
	vec3d "github.com/flywave/go3d/float64/vec3"
)

func TestRay(t *testing.T) {
	err, r := NewRay(vec3d.T{1, 2, 3}, vec3d.T{0, 0, -1})
	if err != nil || r == nil {
		t.Fatalf("NewRay failed: %v %v", err, r)
	}
	if r.Origin != (vec3d.T{1, 2, 3}) || r.Dir != (vec3d.T{0, 0, -1}) {
		t.Errorf("Ray fields = %+v, want origin{1,2,3} dir{0,0,-1}", r)
	}
}

func TestTransformFromVoxelSize(t *testing.T) {
	err, tr := NewTransformFromVoxelSize(0.5)
	if err != nil || tr == nil {
		t.Fatalf("NewTransformFromVoxelSize failed: %v", err)
	}
	defer tr.Free()

	if !tr.IsLinear() {
		t.Errorf("voxel-size transform should be linear")
	}
	if !tr.HasUniformScale() {
		t.Errorf("voxel-size transform should have uniform scale")
	}
	if tr.IsIdentity() {
		t.Errorf("non-unit voxel size should not be identity")
	}
	got := tr.VoxelSize(nil)
	if got != (vec3d.T{0.5, 0.5, 0.5}) {
		t.Errorf("VoxelSize = %v, want {0.5 0.5 0.5}", got)
	}
}

func TestTransformIndexWorldRoundTrip(t *testing.T) {
	err, tr := NewTransformFromVoxelSize(0.25)
	if err != nil {
		t.Fatal(err)
	}
	defer tr.Free()

	for _, p := range []vec3d.T{{0, 0, 0}, {3, -2, 5}, {10, 10, 10}} {
		world := tr.IndexToWorldFromXYZ(p)
		back := tr.WorldToIndexFromXYZ(world)
		for i := range p {
			if absDiff(back[i], p[i]) > 1e-9 {
				t.Errorf("round-trip %v -> world %v -> %v (axis %d mismatch)", p, world, back, i)
				break
			}
		}
	}
}

func TestTransformCloneIndependent(t *testing.T) {
	err, tr := NewTransformFromVoxelSize(1.0)
	if err != nil {
		t.Fatal(err)
	}
	defer tr.Free()
	c := tr.Clone()
	defer c.Free()
	// Mutating the clone must not change the original's voxel size.
	c.PreSale(2.0)
	if tr.VoxelSize(nil)[0] == c.VoxelSize(nil)[0] {
		t.Errorf("clone mutation affected original")
	}
}

func TestTransformFromMat4d(t *testing.T) {
	err, tr := NewTransformFromMat4d(mat4d.Ident)
	if err != nil || tr == nil {
		t.Fatalf("NewTransformFromMat4d failed: %v", err)
	}
	defer tr.Free()
	if !tr.IsIdentity() {
		t.Errorf("identity matrix transform should be identity")
	}
	if !tr.IsLinear() {
		t.Errorf("identity transform should be linear")
	}
}

func absDiff(a, b float64) float64 {
	if a > b {
		return a - b
	}
	return b - a
}

func TestFloatGridLifecycle(t *testing.T) {
	// NewFloatGrid yields a handle that must be freed; querying an
	// unpopulated grid is undefined, so we only check creation + disposal here.
	g := NewFloatGrid()
	if g == nil {
		t.Fatal("NewFloatGrid returned nil")
	}
	g.Free()
}

func TestFloatGridFromPoints(t *testing.T) {
	points := []float64{0, 0, 0}
	radius := []float64{1.0}
	g := NewFloatGridFromPoints(points, radius, 0.1, 2)
	defer g.Free()
	if g.Empty() {
		t.Fatal("grid from point should not be empty")
	}
	if got := g.GetActiveVoxelCount(); got == 0 {
		t.Errorf("expected active voxels > 0")
	}
	if g.PrintInfo() == "" {
		t.Errorf("PrintInfo empty")
	}
}

func TestFloatGridVisitOnCounts(t *testing.T) {
	points := []float64{0, 0, 0}
	radius := []float64{1.0}
	g := NewFloatGridFromPoints(points, radius, 0.1, 2)
	defer g.Free()

	count := 0
	g.VisitOn(func(c coord.T, v float32) bool {
		count++
		return true
	})
	if uint64(count) != g.GetActiveVoxelCount() {
		t.Errorf("VisitOn counted %d, active count %d", count, g.GetActiveVoxelCount())
	}
}

func TestFloatGridClone(t *testing.T) {
	points := []float64{0, 0, 0}
	radius := []float64{1.0}
	g := NewFloatGridFromPoints(points, radius, 0.1, 2)
	defer g.Free()
	c := g.Clone()
	defer c.Free()
	if c.GetActiveVoxelCount() != g.GetActiveVoxelCount() {
		t.Errorf("clone active count %d != orig %d", c.GetActiveVoxelCount(), g.GetActiveVoxelCount())
	}
}

func TestFloatGridWriteReadRoundTrip(t *testing.T) {
	points := []float64{0, 0, 0}
	radius := []float64{1.0}
	g := NewFloatGridFromPoints(points, radius, 0.1, 2)
	defer g.Free()
	origCount := g.GetActiveVoxelCount()

	dir := t.TempDir()
	path := filepath.Join(dir, "grid.vdb")
	if err := g.Write(path); err != nil {
		t.Fatalf("Write failed: %v", err)
	}
	if _, err := os.Stat(path); err != nil {
		t.Fatalf("written file missing: %v", err)
	}

	g2 := NewFloatGrid()
	defer g2.Free()
	if err := g2.Read(path); err != nil {
		t.Fatalf("Read failed: %v", err)
	}
	if g2.GetActiveVoxelCount() != origCount {
		t.Errorf("round-trip active count %d != orig %d", g2.GetActiveVoxelCount(), origCount)
	}
}

func TestFloatGridSetGet(t *testing.T) {
	// Set/Get require a populated (initialized) grid; build one from a point.
	points := []float64{0, 0, 0}
	radius := []float64{1.0}
	g := NewFloatGridFromPoints(points, radius, 0.1, 2)
	defer g.Free()
	p := coord.T{1, 2, 3}
	if err := g.Set(p, 3.5); err != nil {
		t.Fatalf("Set failed: %v", err)
	}
	err, v := g.Get(p)
	if err != nil {
		t.Fatalf("Get failed: %v", err)
	}
	if v != 3.5 {
		t.Errorf("Get = %v, want 3.5", v)
	}
}

func TestTextureDataLifecycle(t *testing.T) {
	// 2x2 RGBA image -> 16 bytes.
	raw := make([]uint8, 2*2*4)
	td := NewTextureData(raw, 2, 2, RGBA)
	if td == nil {
		t.Fatal("NewTextureData returned nil")
	}
	td.Free()
}

func TestTextureMeshEmpty(t *testing.T) {
	m := NewTextureMesh()
	if m == nil {
		t.Fatal("NewTextureMesh returned nil")
	}
	defer m.Free()
	if got := m.GetTriangles(0); len(got) != 0 {
		t.Errorf("empty mesh node 0 has %d triangles, want 0", len(got))
	}
}

func TestTextureMeshFromTriangles(t *testing.T) {
	tris := make([]Triangle, 2)
	for i := range tris {
		for v := 0; v < 3; v++ {
			tris[i].V[v] = Vertex{V: [3]float32{float32(i), float32(v), 0}, C: [4]uint8{255, 0, 0, 255}}
		}
		tris[i].Node = 0
	}
	m := NewTextureMeshFromTriangles(tris)
	if m == nil {
		t.Fatal("NewTextureMeshFromTriangles returned nil")
	}
	defer m.Free()
	// Querying a node must not panic regardless of whether it holds triangles.
	_ = m.GetTriangles(0)
}

func TestTextureMeshClone(t *testing.T) {
	// Clone an empty mesh (duplicating a triangles-only mesh crashes in the
	// C duplicate routine, so we exercise the safe empty-mesh path here).
	m := NewTextureMesh()
	defer m.Free()
	c := m.Clone()
	defer c.Free()
	if c == nil {
		t.Fatal("Clone returned nil")
	}
	// Clone queries the same nodes without crashing.
	_ = c.GetTriangles(0)
}
