package vdb

import (
	"math"
	"testing"

	vec2d "github.com/flywave/go3d/float64/vec2"
)

func TestMinMaxHelpers(t *testing.T) {
	if Min(3, 5) != 3 || Min(5, 3) != 3 || Min(5, 5) != 5 {
		t.Errorf("Min unexpected")
	}
	if Max(3, 5) != 5 || Max(5, 3) != 5 || Max(5, 5) != 5 {
		t.Errorf("Max unexpected")
	}
}

func TestQuadToBufferOffset(t *testing.T) {
	const w, h = uint32(256), uint32(256)
	for _, c := range []struct {
		quad uint32
		want uint32
	}{
		{0, 0},
		{1, w / 2},
		{2, (h * w) / 2},
		{3, ((h + 1) * w) / 2},
	} {
		if got := quadToBufferOffset(c.quad, w, h); got != c.want {
			t.Errorf("quadToBufferOffset(%d) = %d, want %d", c.quad, got, c.want)
		}
	}
}

func TestMagnifyQuadAddr(t *testing.T) {
	for _, c := range []struct {
		quad        uint32
		wantRowCol  [2]uint32
	}{
		{0, [2]uint32{4, 6}},
		{1, [2]uint32{4, 7}},
		{2, [2]uint32{5, 6}},
		{3, [2]uint32{5, 7}},
	} {
		r, col := magnifyQuadAddr(2, 3, c.quad)
		if r != c.wantRowCol[0] || col != c.wantRowCol[1] {
			t.Errorf("magnifyQuadAddr(2,3,%d) = (%d,%d), want %v", c.quad, r, col, c.wantRowCol)
		}
	}
}

func TestTileIndexToStringAndGet(t *testing.T) {
	bits := []byte{0, 1, 2, 3}
	ti := NewTileIndexFromLevelAndBitList(uint32(len(bits)), bits)
	if got := ti.ToString(); got != "0123" {
		t.Errorf("ToString = %q, want %q", got, "0123")
	}
	for i, want := range bits {
		if got := ti.Get(uint32(i)); got != uint32(want) {
			t.Errorf("Get(%d) = %d, want %d", i, got, want)
		}
	}
	if level := ti.GetLevel(); level != uint32(len(bits)) {
		t.Errorf("GetLevel = %d, want %d", level, len(bits))
	}
}

func TestTileIndexFromBitListString(t *testing.T) {
	// NewTileIndexFromBitList takes an ASCII-digit string.
	fromStr := NewTileIndexFromBitList("0123")
	fromBytes := NewTileIndexFromLevelAndBitList(4, []byte{0, 1, 2, 3})
	if !fromStr.Equal(fromBytes) {
		t.Errorf("FromBitList(\"0123\") != FromLevelAndBitList({0,1,2,3}): %v vs %v",
			fromStr.Path(), fromBytes.Path())
	}
}

func TestTileIndexWhichChildRoundtrip(t *testing.T) {
	parent := NewTileIndexFromLevelAndBitList(2, []byte{1, 2})
	for c := uint32(0); c < 4; c++ {
		child := parent.Child(c)
		if got := child.WhichChild(); got != c {
			t.Errorf("child(%d).WhichChild() = %d, want %d", c, got, c)
		}
		if !child.Parent().Equal(parent) {
			t.Errorf("child(%d).Parent() != parent", c)
		}
	}
}

func TestTileIndexAncestry(t *testing.T) {
	parent := NewTileIndexFromLevelAndBitList(2, []byte{1, 2})
	child := parent.Child(3).Child(1)
	if !parent.IsAncestorOf(child) {
		t.Errorf("parent should be ancestor of child")
	}
	if child.IsAncestorOf(parent) {
		t.Errorf("child should not be ancestor of parent")
	}
	// Relative path from parent to child has the level difference.
	rel := RelativePath(parent, child)
	if rel.GetLevel() != child.GetLevel()-parent.GetLevel() {
		t.Errorf("RelativePath level = %d, want %d", rel.GetLevel(), child.GetLevel()-parent.GetLevel())
	}
	// Concatenating the relative path back onto the parent reconstructs the child.
	if got := parent.Concatenate(rel); !got.Equal(child) {
		t.Errorf("Concatenate(relative) != child")
	}
}

func TestTileIndexLessGreater(t *testing.T) {
	a := NewTileIndexFromLevelAndBitList(2, []byte{0, 0})
	b := NewTileIndexFromLevelAndBitList(2, []byte{3, 3})
	// a.Greater(b) must equal b.Less(a).
	if a.Greater(b) != b.Less(a) {
		t.Errorf("Greater/Less inconsistency")
	}
	if a.Equal(b) {
		t.Errorf("distinct tiles reported equal")
	}
}

func TestTileIndexValid(t *testing.T) {
	root := NewTileIndex()
	if !root.Valid() {
		t.Errorf("root tile should be valid")
	}
	ti := NewTileIndexFromLevelAndBitList(1, []byte{0})
	if !ti.Valid() {
		t.Errorf("level-1 tile should be valid")
	}
}

func TestTileIndexIsPostOrder(t *testing.T) {
	parent := NewTileIndexFromLevelAndBitList(1, []byte{0})
	child := parent.Child(2)
	// Post-order visits children before parents.
	if !IsPostOrder(child, parent) {
		t.Errorf("IsPostOrder(child, parent) = false, want true")
	}
	if IsPostOrder(parent, child) {
		t.Errorf("IsPostOrder(parent, child) = true, want false")
	}
}

func TestTileIndexToIndex(t *testing.T) {
	ti := NewTileIndexFromLevelAndBitList(2, []byte{0, 0})
	// ToIndex(level) is a stable, non-negative integer identifier.
	if idx := ti.ToIndex(2); int64(idx) < 0 {
		t.Errorf("ToIndex negative: %d", idx)
	}
}

func TestLonLatMercRoundTrip(t *testing.T) {
	cases := []struct{ lon, lat float64 }{
		{0, 0},
		{10, 20},
		{-30, -15},
		{120, 45},
	}
	const eps = 1e-6
	for _, c := range cases {
		x := []float64{c.lon}
		y := []float64{c.lat}
		if !lonlat2merc(x, y, 1) {
			t.Fatalf("lonlat2merc returned false")
		}
		mx, my := x[0], y[0]
		if !merc2lonlat(x, y, 1) {
			t.Fatalf("merc2lonlat returned false")
		}
		if math.Abs(x[0]-c.lon) > eps || math.Abs(y[0]-c.lat) > eps {
			t.Errorf("round-trip (%.4f,%.4f) -> (%.4f,%.4f) [merc %.2f,%.2f]",
				c.lon, c.lat, x[0], y[0], mx, my)
		}
	}
}

func TestLonLatMercClamping(t *testing.T) {
	// Longitude beyond [-180,180] is clamped.
	x := []float64{9999}
	y := []float64{0}
	lonlat2merc(x, y, 1)
	if x[0] != MAXEXTENT {
		t.Errorf("lon clamp: merc x = %f, want %f", x[0], MAXEXTENT)
	}
	// Latitude beyond +/-MAX_LATITUDE is clamped before projection.
	x = []float64{0}
	y = []float64{9999}
	lonlat2merc(x, y, 1)
	// After clamping to MAX_LATITUDE then projecting, |y| == MAXEXTENT.
	if math.Abs(math.Abs(y[0])-MAXEXTENT) > 1e-3 {
		t.Errorf("lat clamp: merc |y| = %f, want %f", math.Abs(y[0]), MAXEXTENT)
	}
}

func TestNewTileIndexFromBoxNonNil(t *testing.T) {
	// A small geographic bounding box must yield a valid, non-root tile index.
	box := vec2d.Rect{Min: vec2d.T{-1, -1}, Max: vec2d.T{1, 1}}
	ti := NewTileIndexFromBox(box)
	if ti == nil || !ti.Valid() {
		t.Errorf("NewTileIndexFromBox returned invalid tile")
	}
}
