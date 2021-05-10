package vdb

type Space interface {
	ToGridWord(xyz []float64) []float64
	ToSpaceWord(xyz []float64) []float64

	TileToSpace(xyz []float64) []float64
	MakeTileRay(xyz []float64, max []float64) (error, *Ray)

	ComputeDistanceFromSurface(vxyz []float64) float64

	ComputePointFromElevation(xyz []float64, elevation float64) []float64
	TileSize(min []float64, max []float64) float64
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

func (l *Local) ToGridWord(pxyz []float64) []float64 {
	switch l.plane {
	case PT_XY:
		return pxyz

	case PT_XZ:
		return []float64{pxyz[0], pxyz[2], pxyz[1]}

	case PT_YZ:
		return []float64{pxyz[1], pxyz[2], pxyz[0]}
	}
	return nil
}

func (l *Local) ToSpaceWord(xyz []float64) []float64 {
	return l.ToGridWord(xyz)
}

func (l *Local) TileToSpace(xyz []float64) []float64 {
	return []float64{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}
}

func (l *Local) MakeTileRay(xyz []float64, max []float64) (error, *Ray) {
	return NewRay([]float64{0, 0, 0}, []float64{1, 0, 0})
}

func (l *Local) ComputeDistanceFromSurface(vxyz []float64) float64 {
	return vxyz[2]
}

func (l *Local) ComputePointFromElevation(xyz []float64, elevation float64) []float64 {
	return []float64{xyz[0], xyz[1], elevation}
}

func (l *Local) TileSize(min []float64, max []float64) float64 {
	return max[0] - min[0]
}
