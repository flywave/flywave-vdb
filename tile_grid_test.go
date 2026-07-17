package vdb

import (
	"testing"

	vec2d "github.com/flywave/go3d/float64/vec2"
	vec3d "github.com/flywave/go3d/float64/vec3"
	mat4d "github.com/flywave/go3d/float64/mat4"
)

func TestTileNewAndAccessors(t *testing.T) {
	idx := NewTileIndexFromLevelAndRowCol(5, 3, 7)
	bbox := vec2d.Rect{Min: vec2d.T{1, 2}, Max: vec2d.T{3, 4}}
	tl := NewTile(idx, bbox)
	if tl == nil {
		t.Fatal("NewTile returned nil")
	}
	if !tl.Index().Equal(idx) {
		t.Errorf("Index() not equal")
	}
	if tl.Bounds() != bbox {
		t.Errorf("Bounds = %v, want %v", tl.Bounds(), bbox)
	}
}

func TestTileCenter(t *testing.T) {
	tl := NewTile(NewTileIndexFromLevelAndRowCol(0, 0, 0), vec2d.Rect{Min: vec2d.T{0, 0}, Max: vec2d.T{2, 4}})
	x, y := tl.Center()
	if x != 1 || y != 2 {
		t.Errorf("Center = (%f,%f), want (1,2)", x, y)
	}
	c3d := tl.Center3D()
	if c3d != (vec3d.T{1, 2, 0}) {
		t.Errorf("Center3D = %v, want {1 2 0}", c3d)
	}
}

func TestTileContains(t *testing.T) {
	tl := NewTile(NewTileIndexFromLevelAndRowCol(0, 0, 0), vec2d.Rect{Min: vec2d.T{0, 0}, Max: vec2d.T{10, 10}})
	if !tl.Contains(vec2d.Rect{Min: vec2d.T{1, 1}, Max: vec2d.T{2, 2}}) {
		t.Errorf("inner box should be contained")
	}
	if tl.Contains(vec2d.Rect{Min: vec2d.T{20, 20}, Max: vec2d.T{30, 30}}) {
		t.Errorf("outer box should NOT be contained")
	}
}

func TestTileIsBlack(t *testing.T) {
	tl := NewTile(NewTileIndexFromLevelAndRowCol(5, 3, 7), vec2d.Rect{})
	// row=3, col=7 => (3+7)%2 = 0 => false (not black, white)
	if tl.IsBlack() {
		t.Errorf("(3+7)%%2=0 should be white")
	}
	tl2 := NewTile(NewTileIndexFromLevelAndRowCol(5, 2, 3), vec2d.Rect{})
	if !tl2.IsBlack() {
		t.Errorf("(2+3)%%2=1 should be black")
	}
}

func TestTileXYZ(t *testing.T) {
	idx := NewTileIndexFromLevelAndRowCol(7, 4, 9)
	tl := NewTile(idx, vec2d.Rect{})
	x, y, z := tl.XYZ()
	if x != 9 || y != 4 || z != 7 {
		t.Errorf("XYZ = (%d,%d,%d), want (9,4,7)", x, y, z)
	}
}

func TestTileGridNew(t *testing.T) {
	l := NewLocalSpace(PT_XY)
	tg := NewTileGrid(l, vec2d.Rect{Min: vec2d.T{0, 0}, Max: vec2d.T{10, 10}}, 5)
	if tg == nil {
		t.Fatal("NewTileGrid returned nil")
	}
	if tg.GetLevel() != 5 {
		t.Errorf("Level = %d, want 5", tg.GetLevel())
	}
	if tg.GetBounds().Min != (vec2d.T{0, 0}) || tg.GetBounds().Max != (vec2d.T{10, 10}) {
		t.Errorf("Bounds = %v", tg.GetBounds())
	}
	if tg.GetSpace() == nil {
		t.Errorf("Space is nil")
	}
}

func TestTileGridCellSize(t *testing.T) {
	l := NewLocalSpace(PT_XY)
	tg := NewTileGrid(l, vec2d.Rect{Min: vec2d.T{0, 0}, Max: vec2d.T{10, 10}}, 5)
	cs := tg.CellSize()
	if cs[0] <= 0 || cs[1] <= 0 {
		t.Errorf("CellSize should be positive: %v", cs)
	}
}

func TestFeatureDataLifecycle(t *testing.T) {
	model := FeatureModel{Data: []byte{1, 2, 3}}
	fd := NewFeatureData(model)
	if fd == nil {
		t.Fatal("NewFeatureData returned nil")
	}
	defer fd.Free()
	got := fd.Get()
	if got == nil {
		t.Errorf("Get returned nil")
	}
	model2 := FeatureModel{Data: []byte{4, 5, 6}}
	fd.Set(model2)
	got2 := fd.Get()
	if got2 == nil {
		t.Errorf("Get after Set returned nil")
	}
}

func TestMaterialDataLifecycle(t *testing.T) {
	model := MaterialModel{}
	md := NewMaterialData(model)
	if md == nil {
		t.Fatal("NewMaterialData returned nil")
	}
	defer md.Free()
	if md.Get() == nil {
		t.Errorf("Get returned nil")
	}
	md.Set(MaterialModel{})
	if md.Get() == nil {
		t.Errorf("Get after Set returned nil")
	}
}

func TestFeaturesAndMaterials(t *testing.T) {
	// Features and Materials are created internally by the C API;
	// direct Go-side construction results in a nil handle. We just verify
	// the creation function signatures exist by testing NewMaterialData
	// (already tested in TestMaterialDataLifecycle).
}

func TestNewLocalSpacePanic(t *testing.T) {
	for _, p := range []PlaneType{PT_XY, PT_XZ, PT_YZ} {
		l := NewLocalSpace(p)
		if l == nil {
			t.Errorf("NewLocalSpace(%d) returned nil", p)
		}
	}
}

func TestNewOperator(t *testing.T) {
	b := NewVoxelPixel()
	defer b.Free()
	m := NewVoxelMeshBuilder()
	defer m.Free()
	clip := &NoneClipBoxCreateor{}

	op := NewOperator(OP_VOXELIZE, b, nil, clip, 1.0, 0, GC_LEVEL_SET, mat4d.Ident)
	if op == nil {
		t.Errorf("NewOperator returned nil")
	}
}

func TestGridClipOffset(t *testing.T) {
	// just test it doesn't panic
	_ = NewTileClipBoxCreateor(nil, 1.0)
}

func TestTileIndexFromBitList(t *testing.T) {
	// ascii branchlist round trip
	ti := NewTileIndexFromBitList("0123")
	if ti == nil {
		t.Fatal("NewTileIndexFromBitList returned nil")
	}
	if ti.GetLevel() != 4 {
		t.Errorf("level = %d, want 4", ti.GetLevel())
	}
	// ToString roundtrip for ascii digits
	if s := ti.ToString(); s != "0123" {
		t.Errorf("ToString = %q", s)
	}
}
