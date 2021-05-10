package vdb

type TileGrid struct {
	Level       uint16
	PixelWidth  float64
	PixelHeight float64
	Bounds      BBox2d
	Global      Space
}

func NewTileGrid(g Space, bbox BBox2d, level uint16) *TileGrid {
	t := &TileGrid{Global: g, Bounds: bbox, Level: level}
	count := t.allWordGridCount()
	t.PixelWidth = (bbox[2] - bbox[0]) / float64(count)
	t.PixelHeight = (bbox[3] - bbox[1]) / float64(count)
	return t
}

func (t *TileGrid) allWordGridCount() int {
	return int(1 << t.Level)
}

func (t *TileGrid) ToTileCoord(xyz []float64) []uint32 {
	return t.ToTileCoord2d(t.Global.ToGridWord(xyz))
}

func (t *TileGrid) ToTileCoord2d(xy []float64) []uint32 {
	pos := make([]uint32, 2)
	pos[0] += uint32((t.Bounds[2] - t.Bounds[0]) / 2.0)
	pos[1] += uint32((t.Bounds[3] - t.Bounds[1]) / 2.0)
	pos[0] /= uint32(t.PixelWidth)
	pos[1] /= uint32(t.PixelHeight)
	return pos
}

func (t *TileGrid) CreateTile(x, y uint32) *Tile {
	hw := float64((t.Bounds[2] - t.Bounds[0]) / 2.0)
	hh := float64((t.Bounds[3] - t.Bounds[1]) / 2.0)
	box := BBox2d{float64(x)*t.PixelWidth - hw, float64(x+1)*t.PixelWidth - hw, float64(y+1)*t.PixelHeight - hh, float64(y)*t.PixelHeight - hh}
	return NewTile(NewQuadtreePathFromLevelAndRowCol(uint32(t.Level), y, x), box)
}

func (t *TileGrid) CellSize() (width, height float64) {
	width = t.PixelWidth
	height = t.PixelHeight
	return
}
