package vdb

import (
	"math"

	"github.com/flywave/flywave-vdb/coord"
	vec2d "github.com/flywave/go3d/float64/vec2"
	vec3d "github.com/flywave/go3d/float64/vec3"
)

type TileGrid struct {
	level      uint16
	tileWidth  float64
	tileHeight float64
	bounds     vec2d.Rect
	global     Space
}

func NewTileGrid(g Space, bbox vec2d.Rect, level uint16) *TileGrid {
	t := &TileGrid{global: g, bounds: bbox, level: level}
	count := t.allWordGridCount()
	t.tileWidth = (bbox.Max[0] - bbox.Min[0]) / float64(count)
	t.tileHeight = (bbox.Max[1] - bbox.Min[1]) / float64(count)
	return t
}

func (t *TileGrid) GetLevel() uint16 {
	return t.level
}

func (t *TileGrid) GetBounds() vec2d.Rect {
	return t.bounds
}

func (t *TileGrid) GetSpace() Space {
	return t.global
}

func (t *TileGrid) allWordGridCount() int {
	return int(1 << t.level)
}

func (t *TileGrid) ToTileCoord(xyz vec3d.T) coord.T {
	return t.ToTileCoord2d(t.global.ToGridWord(xyz))
}

func (t *TileGrid) ToTileCoord2d(xyz vec3d.T) coord.T {
	hw := (t.bounds.Max[0] - t.bounds.Min[0]) / 2.0
	hh := (t.bounds.Max[1] - t.bounds.Min[1]) / 2.0
	return coord.T{
		int32(math.Floor((xyz[0] + hw) / t.tileWidth)),
		int32(math.Floor((xyz[1] + hh) / t.tileHeight)),
		int32(xyz[2]),
	}
}

func (t *TileGrid) CreateTile(x, y uint32) *Tile {
	hw := float64((t.bounds.Max[0] - t.bounds.Min[0]) / 2.0)
	hh := float64((t.bounds.Max[1] - t.bounds.Min[1]) / 2.0)
	box := vec2d.Rect{Min: vec2d.T{float64(x)*t.tileWidth - hw, float64(y)*t.tileHeight - hh}, Max: vec2d.T{float64(x+1)*t.tileWidth - hw, float64(y+1)*t.tileHeight - hh}}
	return NewTile(NewTileIndexFromLevelAndRowCol(uint32(t.level), y, x), box)
}

func (t *TileGrid) CellSize() vec2d.T {
	return vec2d.T{t.tileWidth, t.tileHeight}
}
