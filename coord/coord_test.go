package coord

import (
	"testing"

	vec3d "github.com/flywave/go3d/float64/vec3"
)

func TestRoundFloorCeilFrom(t *testing.T) {
	in := vec3d.T{1.4, 2.6, -0.5}
	for _, c := range []struct {
		name string
		got  *T
		want T
	}{
		{"RoundFrom", RoundFrom(in), T{1, 3, -1}},
		{"FloorFrom", FloorFrom(in), T{1, 2, -1}},
		{"CeilFrom", CeilFrom(in), T{2, 3, 0}},
	} {
		if *c.got != c.want {
			t.Errorf("%s(%v) = %v, want %v", c.name, in, *c.got, c.want)
		}
	}
}

func TestParseAndString(t *testing.T) {
	for _, s := range []string{"1 2 3", "-5 0 7", "  10 -20 30  "} {
		v, err := Parse(s)
		if err != nil {
			t.Fatalf("Parse(%q) error: %v", s, err)
		}
		// String concatenates the three ints with no separator.
		if got := v.String(); got == "" {
			t.Errorf("String() of %v is empty", v)
		}
	}
	if _, err := Parse("not numbers"); err == nil {
		t.Errorf("Parse(%q) expected error, got nil", "not numbers")
	}
}

func TestAccessors(t *testing.T) {
	v := T{1, 2, 3}
	if v.Rows() != 3 || v.Cols() != 1 || v.Size() != 3 {
		t.Errorf("Rows/Cols/Size = %d/%d/%d, want 3/1/3", v.Rows(), v.Cols(), v.Size())
	}
	if got := v.Get(0, 1); got != 2 {
		t.Errorf("Get(0,1) = %d, want 2", got)
	}
	if got := v.Slice(); len(got) != 3 || got[2] != 3 {
		t.Errorf("Slice() = %v, want [1 2 3]", got)
	}
	zero := T{}
	if !zero.IsZero() {
		t.Errorf("zero vector IsZero() = false, want true")
	}
	nz := T{0, 0, 1}
	if nz.IsZero() {
		t.Errorf("{0,0,1} IsZero() = true, want false")
	}
}

func TestLength(t *testing.T) {
	v := T{3, 4, 0}
	if v.LengthSqr() != 25 {
		t.Errorf("LengthSqr() = %d, want 25", v.LengthSqr())
	}
	if v.Length() != 5 {
		t.Errorf("Length() = %d, want 5", v.Length())
	}
}

func TestScale(t *testing.T) {
	v := T{1, 2, 3}
	// Scaled does not mutate the receiver.
	if got := v.Scaled(2); got != (T{2, 4, 6}) {
		t.Errorf("Scaled(2) = %v, want {2 4 6}", got)
	}
	if v != (T{1, 2, 3}) {
		t.Errorf("Scaled mutated receiver: %v", v)
	}
	// Scale mutates and returns the receiver (a *T); uses Floor.
	v2 := T{1, 2, 3}
	got := v2.Scale(0.5)
	if *got != (T{0, 1, 1}) || v2 != (T{0, 1, 1}) {
		t.Errorf("Scale(0.5) = %v, receiver %v, want {0 1 1}", *got, v2)
	}
}

func TestInvert(t *testing.T) {
	v := T{1, -2, 3}
	if got := v.Inverted(); got != (T{-1, 2, -3}) {
		t.Errorf("Inverted() = %v, want {-1 2 -3}", got)
	}
	if v != (T{1, -2, 3}) {
		t.Errorf("Inverted mutated receiver: %v", v)
	}
	v.Invert()
	if v != (T{-1, 2, -3}) {
		t.Errorf("Invert() receiver = %v, want {-1 2 -3}", v)
	}
}

func TestAbs(t *testing.T) {
	if Abs(-5) != 5 || Abs(5) != 5 || Abs(0) != 0 {
		t.Errorf("Abs(-5)=%d Abs(5)=%d Abs(0)=%d", Abs(-5), Abs(5), Abs(0))
	}
	v := T{-1, 2, -3}
	if got := v.Absed(); got != (T{1, 2, 3}) {
		t.Errorf("Absed() = %v, want {1 2 3}", got)
	}
	if v != (T{-1, 2, -3}) {
		t.Errorf("Absed mutated receiver: %v", v)
	}
	v.Abs()
	if v != (T{1, 2, 3}) {
		t.Errorf("Abs() receiver = %v, want {1 2 3}", v)
	}
}

func TestArithmetic(t *testing.T) {
	a, b := T{1, 2, 3}, T{10, 20, 30}
	for _, c := range []struct {
		name string
		got  T
		want T
	}{
		{"Add", Add(&a, &b), T{11, 22, 33}},
		{"Sub", Sub(&a, &b), T{-9, -18, -27}},
		{"Mul", Mul(&a, &b), T{10, 40, 90}},
	} {
		if c.got != c.want {
			t.Errorf("%s(%v,%v) = %v, want %v", c.name, a, b, c.got, c.want)
		}
	}
	// Methods mutate the receiver.
	m := T{1, 2, 3}
	m.Add(&T{1, 1, 1})
	if m != (T{2, 3, 4}) {
		t.Errorf("Add method receiver = %v, want {2 3 4}", m)
	}
	m.Sub(&T{1, 1, 1})
	if m != (T{1, 2, 3}) {
		t.Errorf("Sub method receiver = %v, want {1 2 3}", m)
	}
	m.Mul(&T{2, 2, 2})
	if m != (T{2, 4, 6}) {
		t.Errorf("Mul method receiver = %v, want {2 4 6}", m)
	}
}

func TestMinMax(t *testing.T) {
	a, b := T{1, 5, 3}, T{4, 2, 6}
	if Min(&a, &b) != (T{1, 2, 3}) {
		t.Errorf("Min = %v", Min(&a, &b))
	}
	if Max(&a, &b) != (T{4, 5, 6}) {
		t.Errorf("Max = %v", Max(&a, &b))
	}
}

func TestClamp(t *testing.T) {
	lo, hi := T{0, 0, 0}, T{5, 5, 5}
	v := T{-1, 3, 9}
	if got := v.Clamped(&lo, &hi); got != (T{0, 3, 5}) {
		t.Errorf("Clamped = %v, want {0 3 5}", got)
	}
	if v != (T{-1, 3, 9}) {
		t.Errorf("Clamped mutated receiver: %v", v)
	}
	v.Clamp(&lo, &hi)
	if v != (T{0, 3, 5}) {
		t.Errorf("Clamp receiver = %v, want {0 3 5}", v)
	}
}

func TestBoxBasics(t *testing.T) {
	box, err := ParseBox("1 2 3 4 5 6")
	if err != nil {
		t.Fatalf("ParseBox error: %v", err)
	}
	want := Box{Min: T{1, 2, 3}, Max: T{4, 5, 6}}
	if box != want {
		t.Errorf("ParseBox = %+v, want %+v", box, want)
	}
	if got := FromSlice([]int32{1, 2, 3, 4, 5, 6}); *got != want {
		t.Errorf("FromSlice = %+v, want %+v", *got, want)
	}
	arr := box.Array()
	if *arr != [...]int32{1, 2, 3, 4, 5, 6} {
		t.Errorf("Array = %v", *arr)
	}
	if len(box.Slice()) != 6 {
		t.Errorf("Slice len = %d, want 6", len(box.Slice()))
	}
}

func TestBoxContainsCenterDiagonal(t *testing.T) {
	box := Box{Min: T{0, 0, 0}, Max: T{2, 2, 2}}
	if !box.ContainsCoord(&T{1, 1, 1}) {
		t.Errorf("ContainsCoord({1,1,1}) = false, want true")
	}
	if box.ContainsCoord(&T{3, 3, 3}) {
		t.Errorf("ContainsCoord({3,3,3}) = true, want false")
	}
	// Boundary is inclusive.
	if !box.ContainsCoord(&T{0, 0, 0}) || !box.ContainsCoord(&T{2, 2, 2}) {
		t.Errorf("boundary not inclusive")
	}
	if got := box.Diagonal(); got != (T{2, 2, 2}) {
		t.Errorf("Diagonal = %v, want {2 2 2}", got)
	}
	// Center = Add then Scale(0.5) with Floor -> {2,2,2}*0.5 floored = {1,1,1}.
	if got := box.Center(); got != (T{1, 1, 1}) {
		t.Errorf("Center = %v, want {1 1 1}", got)
	}
}

func TestBoxIntersects(t *testing.T) {
	a := Box{Min: T{0, 0, 0}, Max: T{2, 2, 2}}
	for _, c := range []struct {
		name string
		other Box
		want  bool
	}{
		{"identical", a, true},
		{"inside", Box{Min: T{0, 0, 0}, Max: T{1, 1, 1}}, true},
		{"touching", Box{Min: T{2, 2, 2}, Max: T{4, 4, 4}}, true},
		{"far", Box{Min: T{10, 10, 10}, Max: T{12, 12, 12}}, false},
	} {
		if got := a.Intersects(&c.other); got != c.want {
			t.Errorf("Intersects(%s %+v) = %v, want %v", c.name, c.other, got, c.want)
		}
	}
}

func TestBoxJoinExtendJoined(t *testing.T) {
	a := Box{Min: T{0, 0, 0}, Max: T{2, 2, 2}}
	b := Box{Min: T{1, 1, 1}, Max: T{3, 3, 3}}
	// Joined does not mutate inputs.
	got := Joined(&a, &b)
	if got.Min != (T{0, 0, 0}) || got.Max != (T{3, 3, 3}) {
		t.Errorf("Joined = %+v, want Min{0} Max{3}", got)
	}
	// Join mutates receiver.
	j := a
	j.Join(&b)
	if j.Min != (T{0, 0, 0}) || j.Max != (T{3, 3, 3}) {
		t.Errorf("Join receiver = %+v", j)
	}
	// Extend enlarges to contain a point.
	e := Box{Min: T{0, 0, 0}, Max: T{1, 1, 1}}
	e.Extend(&T{5, -2, 1})
	if e.Min != (T{0, -2, 0}) || e.Max != (T{5, 1, 1}) {
		t.Errorf("Extend receiver = %+v", e)
	}
}

func TestSentinels(t *testing.T) {
	if !Zero.IsZero() {
		t.Errorf("Zero not zero")
	}
	if MinVal[0] >= 0 || MaxVal[0] <= 0 {
		t.Errorf("MinVal/MaxVal unexpected: %v / %v", MinVal, MaxVal)
	}
	// MaxBox spans all representable space; any finite coord is contained.
	if !MaxBox.ContainsCoord(&T{0, 0, 0}) {
		t.Errorf("MaxBox does not contain origin")
	}
}
