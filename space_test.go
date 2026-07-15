package vdb

import (
	"testing"

	vec3d "github.com/flywave/go3d/float64/vec3"
)

func TestLocalSpaceToGridWord(t *testing.T) {
	in := vec3d.T{1, 2, 3}
	for _, c := range []struct {
		plane PlaneType
		want  vec3d.T
	}{
		{PT_XY, vec3d.T{1, 2, 3}},
		{PT_XZ, vec3d.T{1, 3, 2}},
		{PT_YZ, vec3d.T{2, 3, 1}},
	} {
		l := NewLocalSpace(c.plane)
		if got := l.ToGridWord(in); got != c.want {
			t.Errorf("plane %v ToGridWord = %v, want %v", c.plane, got, c.want)
		}
		// ToSpaceWord mirrors ToGridWord for the local space.
		if got := l.ToSpaceWord(in); got != c.want {
			t.Errorf("plane %v ToSpaceWord = %v, want %v", c.plane, got, c.want)
		}
	}
}

func TestLocalSpaceElevation(t *testing.T) {
	l := NewLocalSpace(PT_XY)
	// ComputeDistanceFromSurface returns the z component.
	if got := l.ComputeDistanceFromSurface(vec3d.T{1, 2, 5}); got != 5 {
		t.Errorf("ComputeDistanceFromSurface = %v, want 5", got)
	}
	// ComputePointFromElevation keeps x,y and sets z to elevation.
	if got := l.ComputePointFromElevation(vec3d.T{1, 2, 99}, 7); got != (vec3d.T{1, 2, 7}) {
		t.Errorf("ComputePointFromElevation = %v, want {1 2 7}", got)
	}
}

func TestLocalSpaceTileSize(t *testing.T) {
	l := NewLocalSpace(PT_XY)
	if got := l.TileSize(vec3d.T{0, 0, 0}, vec3d.T{5, 9, 0}); got != 5 {
		t.Errorf("TileSize = %v, want 5 (uses x extent)", got)
	}
}

func TestLocalSpaceTileToSpaceIsIdentity(t *testing.T) {
	l := NewLocalSpace(PT_XY)
	m := l.TileToSpace(vec3d.T{3, 4, 5})
	// TileToSpace returns the identity matrix for the local space.
	if m[0][0] != 1 || m[1][1] != 1 || m[2][2] != 1 || m[3][3] != 1 {
		t.Errorf("TileToSpace not identity: %v", m)
	}
}
