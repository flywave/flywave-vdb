package vdb

const (
	EPS = float64(0.000000001)
)

type Tile struct {
	id   *QuadtreePath
	bbox BBox2d
}

func NewTile(id *QuadtreePath, bbox BBox2d) *Tile {
	return &Tile{id: id, bbox: bbox}
}

func (t *Tile) Pos() (x, y float64) {
	x = (t.bbox[0] + t.bbox[1]) / 2
	y = (t.bbox[2] + t.bbox[3]) / 2
	return
}

func (t *Tile) ID() *QuadtreePath {
	return t.id
}

func (t *Tile) BBox() BBox2d {
	return t.bbox
}

func (t *Tile) ZYX() (z, y, x uint32) {
	return t.id.GetLevelRowCol()
}

func (t *Tile) IsBlack() bool {
	_, x, y := t.ZYX()
	return ((x + y) % 2) > 0
}

func (t *Tile) Contains(box BBox2d) bool {
	return t.bbox.Contains(box[:], EPS)
}
