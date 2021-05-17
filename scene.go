package vdb

import (
	mat4d "github.com/flywave/go3d/float64/mat4"
	vec2d "github.com/flywave/go3d/float64/vec2"
	vec3d "github.com/flywave/go3d/float64/vec3"
)

type Scene struct {
	TileGrid
	opt        Options
	space      Space
	storage    Storage
	tiles      map[uint64]Tile
	voxelCache *Cache
}

func NewScene(g Space, extent vec2d.Rect, level uint16, s Storage, opt Options, needInit bool) *Scene {
	cache := NewCache(opt.MaxCacheSize)
	scene := &Scene{TileGrid: *NewTileGrid(g, extent, level), storage: s, opt: opt, voxelCache: cache}
	if needInit && !scene.init() {
		return nil
	}
	cache.OnMiss(func(id string) (Cacheable, error) {
		tid := NewTileIndexFromBitList(id)
		tile := scene.GetTileFromIndex(tid)
		return NewVoxelTileWithStorage(tile, scene.storage), nil
	})
	return scene
}

func (s *Scene) Collect() error {
	return s.voxelCache.Collect()
}

func (s *Scene) init() bool {
	s.tiles = make(map[uint64]Tile)
	tiles := s.storage.ListTiles(s.bounds, s.CellSize())
	if tiles == nil {
		return false
	}
	for _, t := range tiles {
		s.tiles[t.Index().Path()] = t
	}
	return true
}

func (s *Scene) Contains(id *TileIndex) bool {
	_, ok := s.tiles[id.Path()]
	return ok
}

func (s *Scene) GetTileFromIndex(id *TileIndex) *Tile {
	t, ok := s.tiles[id.Path()]
	if ok {
		return &t
	}
	return nil
}

func (s *Scene) GetTile(z, x, y uint32) *Tile {
	id := NewTileIndexFromLevelAndRowCol(z, y, x)
	return s.GetTileFromIndex(id)
}

func (s *Scene) GetVoxelTile(t *Tile) *VoxelTile {
	tile, ok := s.tiles[t.Index().Path()]
	if !ok {
		return nil
	}
	vt, err := s.voxelCache.Get(tile.id.ToString())
	if err != nil {
		return nil
	}
	return vt.(*VoxelTile)
}

func (s *Scene) GetTiles() []Tile {
	tiles := make([]Tile, 0, len(s.tiles))
	for _, v := range tiles {
		tiles = append(tiles, v)
	}
	return tiles
}

type PointQueryTask struct {
	TileTask
	tile     *VoxelTile
	points   []vec3d.T
	surfaces []vec3d.T
	space    Space
}

func (t *PointQueryTask) Run() {
	t.surfaces = make([]vec3d.T, len(t.points))
	wpos := t.space.ToSpaceWord(t.tile.Tile.Pos())
	mat := t.space.TileToSpace(wpos)
	mat_inv := mat.Inverted()
	for i := range t.points {
		t.points[i] = mat_inv.MulVec3(&t.points[i])
	}
	surfaces := t.tile.SurfaceQuery(t.points, t.space)
	for i := range surfaces {
		surfaces[i] = mat.MulVec3(&surfaces[i])
	}
	t.surfaces = surfaces
}

type PointQueryMapped map[uint64]PointQueryTask

func (s *Scene) mapPointQueryTask(points []vec3d.T) PointQueryMapped {
	tmap := make(PointQueryMapped)
	for i := range points {
		coord := s.ToTileCoord(points[i])
		tile := s.CreateTile(uint32(coord[0]), uint32(coord[1]))
		_, ok := tmap[tile.Index().path]
		if !ok {
			tmap[tile.Index().path] = PointQueryTask{tile: s.GetVoxelTile(tile), points: []vec3d.T{points[i]}, space: s.space}
		} else {
			rpoints := tmap[tile.Index().path].points
			rpoints = append(rpoints, points[i])
		}
	}
	return tmap
}

type RangeQueryTask struct {
	TileTask
	tile  *VoxelTile
	box   vec3d.Box
	out   vec3d.Box
	space Space
}

func (t *RangeQueryTask) Run() {
	wpos := t.space.ToSpaceWord(t.tile.Tile.Pos())
	mat := t.space.TileToSpace(wpos)
	mat_inv := mat.Inverted()

	t.box.Min = mat_inv.MulVec3(&t.box.Min)
	t.box.Max = mat_inv.MulVec3(&t.box.Max)

	out := t.tile.vpixel.EvalMaxMinElevation(&t.box)

	out.Min = mat.MulVec3(&out.Min)
	out.Max = mat.MulVec3(&out.Max)
}

type RangeQueryMapped map[uint64]RangeQueryTask

func (s *Scene) mapRangeQueryTask(in *vec3d.Box) RangeQueryMapped {
	min := s.ToTileCoord(in.Min)
	max := s.ToTileCoord(in.Max)

	var tiles []Tile
	for x := min[0]; x <= max[0]; x++ {
		for y := min[1]; y <= max[1]; y++ {
			tiles = append(tiles, *s.CreateTile(uint32(x), uint32(y)))
		}
	}
	tmap := make(RangeQueryMapped)
	for i := range tiles {
		_, ok := tmap[tiles[i].Index().path]
		if !ok {
			tmap[tiles[i].Index().path] = RangeQueryTask{tile: s.GetVoxelTile(&tiles[i]), box: *in, space: s.space}
		}
	}
	return tmap
}

type OperatorTask struct {
	TileTask
	tile *VoxelTile
	mesh *VoxelMesh
	op   Operator
}

func (t *OperatorTask) post() {
	t.tile.Flush()
	t.mesh.Free()
}

func (t *OperatorTask) Run() {
	t.post()
}

type OperatorMapped map[uint64]OperatorTask

func (s *Scene) mapOperatorTask(tp OperatorType, mesh *VoxelMesh, matrix mat4d.T, localFeature uint16) OperatorMapped {
	in := mesh.BoundsInWord(matrix)

	min := s.ToTileCoord(in.Min)
	max := s.ToTileCoord(in.Max)

	var tiles []Tile
	for x := min[0]; x <= max[0]; x++ {
		for y := min[1]; y <= max[1]; y++ {
			tiles = append(tiles, *s.CreateTile(uint32(x), uint32(y)))
		}
	}

	tmap := make(OperatorMapped)
	for i := range tiles {
		_, ok := tmap[tiles[i].Index().path]
		if !ok {
			base := s.GetVoxelTile(&tiles[i])
			clip := NewTileClipBoxCreateor(&tiles[i], s.opt.ClipOffset)
			tmap[tiles[i].Index().path] = OperatorTask{tile: base, mesh: mesh, op: NewOperator(tp, base.GetVoxelPixel(), mesh, clip, s.opt.Precision, localFeature, GC_LEVEL_SET, matrix)}
		}
	}
	return tmap
}
