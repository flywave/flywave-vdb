package vdb

import (
	vec2d "github.com/flywave/go3d/float64/vec2"
)

const (
	EPS = float64(0.000000001)
)

type Tile struct {
	id   *TileIndex
	bbox vec2d.Rect
}

func NewTile(id *TileIndex, bbox vec2d.Rect) *Tile {
	return &Tile{id: id, bbox: bbox}
}

func (t *Tile) Center() (x, y float64) {
	x = (t.bbox.Min[0] + t.bbox.Max[0]) / 2
	y = (t.bbox.Min[1] + t.bbox.Max[1]) / 2
	return
}

func (t *Tile) Index() *TileIndex {
	return t.id
}

func (t *Tile) Bounds() vec2d.Rect {
	return t.bbox
}

func (t *Tile) XYZ() (uint32, uint32, uint32) {
	z, y, x := t.id.GetLevelRowCol()
	return x, y, z
}

func (t *Tile) IsBlack() bool {
	x, y, _ := t.XYZ()
	return ((x + y) % 2) > 0
}

func (t *Tile) Contains(box vec2d.Rect) bool {
	return t.bbox.Contains(&box)
}
