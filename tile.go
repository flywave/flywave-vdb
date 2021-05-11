package vdb

const (
	EPS = float64(0.000000001)
)

type Tile struct {
	id   *TileIndex
	bbox BBox2d
}

func NewTile(id *TileIndex, bbox BBox2d) *Tile {
	return &Tile{id: id, bbox: bbox}
}

func (t *Tile) Center() (x, y float64) {
	x = (t.bbox[0] + t.bbox[1]) / 2
	y = (t.bbox[2] + t.bbox[3]) / 2
	return
}

func (t *Tile) Index() *TileIndex {
	return t.id
}

func (t *Tile) Bounds() BBox2d {
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

func (t *Tile) Contains(box BBox2d) bool {
	return t.bbox.Contains(box[:], EPS)
}
