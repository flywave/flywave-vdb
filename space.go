package vdb

import (
	mat4d "github.com/flywave/go3d/float64/mat4"
	vec3d "github.com/flywave/go3d/float64/vec3"
)

type Space interface {
	ToGridWord(xyz vec3d.T) vec3d.T
	ToSpaceWord(xyz vec3d.T) vec3d.T
	TileToSpace(xyz vec3d.T) mat4d.T
	MakeTileRay(xyz vec3d.T, max vec3d.T) (error, *Ray)
	ComputeDistanceFromSurface(vxyz vec3d.T) float64
	ComputePointFromElevation(xyz vec3d.T, elevation float64) vec3d.T
	TileSize(min vec3d.T, max vec3d.T) float64
}

type PlaneType uint32

const (
	PT_XY = PlaneType(0)
	PT_XZ = PlaneType(1)
	PT_YZ = PlaneType(2)
)

type Local struct {
	Space
	plane PlaneType
}

func NewLocalSpace(plane PlaneType) *Local {
	return &Local{plane: plane}
}

func (l *Local) ToGridWord(pxyz vec3d.T) vec3d.T {
	switch l.plane {
	case PT_XY:
		return pxyz
	case PT_XZ:
		return vec3d.T{pxyz[0], pxyz[2], pxyz[1]}
	case PT_YZ:
		return vec3d.T{pxyz[1], pxyz[2], pxyz[0]}
	}
	return vec3d.Zero
}

func (l *Local) ToSpaceWord(xyz vec3d.T) vec3d.T {
	return l.ToGridWord(xyz)
}

func (l *Local) TileToSpace(xyz vec3d.T) mat4d.T {
	return mat4d.Ident
}

func (l *Local) MakeTileRay(xyz vec3d.T, max vec3d.T) (error, *Ray) {
	return NewRay(vec3d.Zero, vec3d.UnitX)
}

func (l *Local) ComputeDistanceFromSurface(vxyz vec3d.T) float64 {
	return vxyz[2]
}

func (l *Local) ComputePointFromElevation(xyz vec3d.T, elevation float64) vec3d.T {
	return vec3d.T{xyz[0], xyz[1], elevation}
}

func (l *Local) TileSize(min vec3d.T, max vec3d.T) float64 {
	return max[0] - min[0]
}
