package vdb

import (
	mat4d "github.com/flywave/go3d/float64/mat4"
	vec3d "github.com/flywave/go3d/float64/vec3"
)

type Task interface {
	Run() error
}

type TileTask struct {
	Task
	tile         []*VoxelTile
	op           OperatorType
	src          *VoxelMesh
	matToWord    mat4d.T
	boundsInWord vec3d.Box
	precision    float64
	tp           GridClass
}

func NewTileTask(tiles []*VoxelTile, op OperatorType, src *VoxelMesh, matToWord mat4d.T, boundsInWord vec3d.Box) *TileTask {
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

func (t *TileTask) GetMatToWord() mat4d.T {
	return t.matToWord
}

func (t *TileTask) GetBoundsInWord() []float64 {
	return t.boundsInWord.Slice()
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
