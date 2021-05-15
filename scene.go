package vdb

import (
	vec2d "github.com/flywave/go3d/float64/vec2"
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
