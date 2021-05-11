package vdb

type Action interface {
	Run() error
}

type TileAction struct {
	Action
	tile         []*VoxelTile
	op           OperatorType
	src          *VoxelMesh
	matToWord    []float64
	boundsInWord *BBox
}

func (t *TileAction) Free() {
	t.src.Free()
}

func (t *TileAction) Flush() {
	for _, tile := range t.tile {
		tile.SetDirty()
		tile.Flush()
	}
	t.Free()
}

func (t *TileAction) Run() error {
	defer t.Flush()
	return nil
}
