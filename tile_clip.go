package vdb

import (
	mat4d "github.com/flywave/go3d/float64/mat4"
	vec3d "github.com/flywave/go3d/float64/vec3"
)

type TileClipBoxCreateor struct {
	ClipBoxCreateor
	tile       *Tile
	precision  float64
	mat        mat4d.T
	clipOffset float64
}

func NewTileClipBoxCreateor(tile *Tile, precision float64, mat mat4d.T, clipOffset float64) *TileClipBoxCreateor {
	return &TileClipBoxCreateor{tile: tile, precision: precision, mat: mat, clipOffset: clipOffset}
}

func (t *TileClipBoxCreateor) Gen(vertex *FloatGrid, resolution *Transform, sbox *vec3d.Box, box *vec3d.Box) bool {
	modelBBox := resolution.IndexToWorldFromBBox(sbox)

	cbox := t.tile.Bounds()
	width := cbox.Width() / 2.0
	height := cbox.Height() / 2.0

	offset := resolution.IndexToWorldFromXYZ(vec3d.T{t.clipOffset, t.clipOffset, t.clipOffset})

	ew := width + offset[0]
	eh := height + offset[1]

	clipBox := &vec3d.Box{Min: vec3d.T{modelBBox.Min[0] - 1, float64(-ew), float64(-eh)}, Max: vec3d.T{modelBBox.Max[0] + 1, float64(ew), float64(eh)}}
	extBox := &vec3d.Box{Min: vec3d.T{modelBBox.Min[0] - 1, float64(-width), float64(-height)}, Max: vec3d.T{modelBBox.Max[0] + 1, float64(width), float64(height)}}

	if extBox.Contains(clipBox) {
		return false
	}
	*box = vec3d.Joined(clipBox, modelBBox)
	return true
}
