package vdb

import (
	vec2d "github.com/flywave/go3d/float64/vec2"
	vec3d "github.com/flywave/go3d/float64/vec3"
)

type TileGrid struct {
	level      uint16
	tileWidth  float64
	tileHeight float64
	bounds     BBox2d
	global     Space
}

func NewTileGrid(g Space, bbox BBox2d, level uint16) *TileGrid {
	t := &TileGrid{global: g, bounds: bbox, level: level}
	count := t.allWordGridCount()
	t.tileWidth = (bbox[2] - bbox[0]) / float64(count)
	t.tileHeight = (bbox[3] - bbox[1]) / float64(count)
	return t
}

func (t *TileGrid) GetLevel() uint16 {
	return t.level
}

func (t *TileGrid) GetBounds() BBox2d {
	return t.bounds
}

func (t *TileGrid) GetSpace() Space {
	return t.global
}

func (t *TileGrid) allWordGridCount() int {
	return int(1 << t.level)
}

func (t *TileGrid) ToTileCoord(xyz vec3d.T) []uint32 {
	return t.ToTileCoord2d(t.global.ToGridWord(xyz))
}

func (t *TileGrid) ToTileCoord2d(xyz vec3d.T) []uint32 {
	pos := make([]uint32, 2)
	pos[0] += uint32((t.bounds[2] - t.bounds[0]) / 2.0)
	pos[1] += uint32((t.bounds[3] - t.bounds[1]) / 2.0)
	pos[0] /= uint32(t.tileWidth)
	pos[1] /= uint32(t.tileHeight)
	return pos
}

func (t *TileGrid) CreateTile(x, y uint32) *Tile {
	hw := float64((t.bounds[2] - t.bounds[0]) / 2.0)
	hh := float64((t.bounds[3] - t.bounds[1]) / 2.0)
	box := BBox2d{float64(x)*t.tileWidth - hw, float64(x+1)*t.tileWidth - hw, float64(y+1)*t.tileHeight - hh, float64(y)*t.tileHeight - hh}
	return NewTile(NewTileIndexFromLevelAndRowCol(uint32(t.level), y, x), box)
}

func (t *TileGrid) CellSize() vec2d.T {
	return vec2d.T{t.tileWidth, t.tileHeight}
}
