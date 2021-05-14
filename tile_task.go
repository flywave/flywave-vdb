package vdb

type Task interface {
	Run() error
}

type TileTask struct {
	Task
	tile         []*VoxelTile
	op           OperatorType
	src          *VoxelMesh
	matToWord    []float64
	boundsInWord *BBox
	precision    float64
	tp           GridClass
}

func NewTileTask(tiles []*VoxelTile, op OperatorType, src *VoxelMesh, matToWord []float64, boundsInWord *BBox) *TileTask {
	return &TileTask{tile: tiles, op: op, src: src, matToWord: matToWord, boundsInWord: boundsInWord}
}

func (t *TileTask) GetOperatorType() OperatorType {
	return t.op
}

func (t *TileTask) GetVoxelTile() []*VoxelTile {
	return t.tile
}

func (t *TileTask) GetMesh() *VoxelMesh {
	return t.src
}

func (t *TileTask) GetMatToWord() []float64 {
	return t.matToWord
}

func (t *TileTask) GetBoundsInWord() []float64 {
	return t.boundsInWord.GetSlice()
}

func (t *TileTask) Free() {
	t.src.Free()
}

func (t *TileTask) flush() {
	for _, tile := range t.tile {
		tile.SetDirty()
		tile.Flush()
	}
	t.Free()
}

func (t *TileTask) Run() error {
	defer t.flush()
	ops := t.mapOperator()
	for i := range ops {
		ops[i].Apply()
	}
	return nil
}

func (t *TileTask) mapOperator() []Operator {
	ops := make([]Operator, len(t.tile))
	for i := range t.tile {
		ops[i] = t.newOperator()
	}
	return ops
}

func (t *TileTask) newOperator() Operator {
	return nil
}
