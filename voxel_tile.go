package vdb

type VoxelTile struct {
	Cacheable
	SizeAware
	NotifyPurge
	Tile
	vpixel  *VoxelPixel
	dirty   bool
	storage Storage
}

func NewVoxelTileWithStorage(tile *Tile, storage Storage) *VoxelTile {
	return &VoxelTile{Tile: *tile, vpixel: nil, storage: storage, dirty: false}
}

func NewVoxelTile(tile *Tile, p *VoxelPixel) *VoxelTile {
	return &VoxelTile{Tile: *tile, vpixel: p, dirty: true}
}

func NewVoxelTileWithBBox(id *TileIndex, bbox BBox2d, p *VoxelPixel) *VoxelTile {
	return &VoxelTile{Tile: *NewTile(id, bbox), vpixel: p, dirty: true}
}

func (t *VoxelTile) OnPurge(why PurgeReason) {
	defer t.free()
	if t.Vaild() && t.dirty {
		t.Flush()
	}
}

func (t *VoxelTile) Size() int64 {
	return t.vpixel.Size()
}

func (t *VoxelTile) free() {
	if t.vpixel != nil {
		t.vpixel.Free()
	}
	t.vpixel = nil
}

func (t *VoxelTile) RayTest(ray *Ray) (bool, []float64) {
	return t.vpixel.RayTest(ray)
}

func (t *VoxelTile) SurfaceQuery(points [][]float64, space Space) [][]float64 {
	rays := make([]Ray, len(points))
	max := t.EvalBoundingBox()[3:]
	for i, pts := range points {
		_, rt := space.MakeTileRay(pts, max)
		rays[i] = *rt
	}
	res := t.vpixel.RayTests(rays)

	return res
}

func (t *VoxelTile) Vaild() bool {
	return t.vpixel != nil && t.vpixel.Vaild()
}

func (t *VoxelTile) Load() bool {
	if t.storage.TileExists(&t.Tile) {
		return t.storage.ReadTile(t)
	}
	return false
}

func (t *VoxelTile) Flush() bool {
	if t.dirty {
		return t.storage.WriteTile(t)
	}
	return true
}

func (t *VoxelTile) ModTime() int64 {
	return t.storage.TileModTime(&t.Tile)
}

func (t *VoxelTile) GetTile() *Tile {
	return &t.Tile
}

func (t *VoxelTile) GetVoxelPixel() *VoxelPixel {
	return t.vpixel
}

func (t *VoxelTile) SetVoxelPixel(vp *VoxelPixel) {
	t.vpixel = vp
}

func (t *VoxelTile) GetStorage() Storage {
	return t.storage
}

func (t *VoxelTile) IsDirty() bool {
	return t.dirty
}

func (t *VoxelTile) SetDirty() {
	t.dirty = true
}

func (t *VoxelTile) EvalBoundingBox() []float64 {
	if t.vpixel == nil {
		return []float64{0, 0, 0, 0, 0, 0}
	}
	_, box := t.vpixel.GetVoxelGrid().EvalActiveBoundingBox()
	_, in := NewBBox(box)
	out := t.vpixel.VoxelResolution().IndexToWorldFromBBox(in)
	return out.GetSlice()
}
