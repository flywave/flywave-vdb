package vdb

type TileClipBoxCreateor struct {
	ClipBoxCreateor
	tile       *Tile
	precision  float64
	mat        []float64
	clipOffset float64
}

func NewTileClipBoxCreateor(tile *Tile, precision float64, mat []float64, clipOffset float64) *TileClipBoxCreateor {
	return &TileClipBoxCreateor{tile: tile, precision: precision, mat: mat, clipOffset: clipOffset}
}

func (t *TileClipBoxCreateor) Gen(vertex *FloatGrid, resolution *Transform, sbox *BBox, box *BBox) bool {
	modelBBox := resolution.IndexToWorldFromBBox(sbox)

	cbox := t.tile.Bounds()
	width := cbox.Width() / 2.0
	height := cbox.Height() / 2.0

	offset := resolution.IndexToWorldFromXYZ([]float64{t.clipOffset, t.clipOffset, t.clipOffset})

	ew := width + offset[0]
	eh := height + offset[1]

	_, clipBox := NewBBox([]float64{modelBBox.GetMin()[0] - 1, float64(-ew), float64(-eh), modelBBox.GetMax()[0] + 1, float64(ew), float64(eh)})
	_, extBox := NewBBox([]float64{modelBBox.GetMin()[0] - 1, float64(-width), float64(-height), modelBBox.GetMax()[0] + 1, float64(width), float64(height)})

	if extBox.Contains(clipBox) {
		return false
	}
	box.Set(clipBox.Intersect(modelBBox))
	return true
}
