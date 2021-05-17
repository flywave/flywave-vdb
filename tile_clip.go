package vdb

import (
	vec3d "github.com/flywave/go3d/float64/vec3"
)

type TileClipBoxCreateor struct {
	ClipBoxCreateor
	tile       *Tile
	clipOffset float64
}

func NewTileClipBoxCreateor(tile *Tile, clipOffset float64) ClipBoxCreateor {
	return &TileClipBoxCreateor{tile: tile, clipOffset: clipOffset}
}

func (t *TileClipBoxCreateor) Gen(vertex *FloatGrid, resolution *Transform, sbox vec3d.Box, box *vec3d.Box) bool {
	modelBBox := resolution.IndexToWorldFromBBox(&sbox)

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
