package vdb

import (
	"os"
	"path/filepath"
	"regexp"
	"strconv"
	"strings"

	vec2d "github.com/flywave/go3d/float64/vec2"
)

const (
	VOXEL_TILE_EXT        = string(".vtile")
	VOXEL_TILE_EXT_TEMP   = string(".vtile.tmp")
	VOXEL_TILE_EXT_REGEXP = string(`^([\d]+)_([\d]+)_([\d]+)`) + VOXEL_TILE_EXT + string('$')
)

var (
	_parsr_file_reg = regexp.MustCompile(VOXEL_TILE_EXT_REGEXP)
)

type Storage interface {
	WriteTile(tile *VoxelTile) bool
	ReadTile(tile *VoxelTile) bool
	TileExists(tile *Tile) bool
	RemoveTile(tile *Tile) bool
	ListTiles(bbox vec2d.Rect, tileSize vec2d.T) []Tile
	UpdateTiles(tiles []*VoxelTile) bool
	TileModTime(tile *Tile) int64
}

func MakeVoxelTileName(root string, tile *Tile, extensions string) string {
	x, y, z := tile.XYZ()
	return root + "/" + strconv.Itoa(int(z)) + "_" + strconv.Itoa(int(x)) + "_" + strconv.Itoa(int(y)) + extensions
}

func ParseVoxelTileNameToTile(filename string, bounds vec2d.Rect, tileSize vec2d.T) *Tile {
	zxystr := _parsr_file_reg.FindStringSubmatch(filename)
	if len(zxystr) != 4 {
		return nil
	}
	z, err := strconv.Atoi(zxystr[1])
	if err != nil {
		return nil
	}
	x, err := strconv.Atoi(zxystr[2])
	if err != nil {
		return nil
	}
	y, err := strconv.Atoi(zxystr[3])
	if err != nil {
		return nil
	}
	hw := float64((bounds.Max[0] - bounds.Min[0]) / 2.0)
	hh := float64((bounds.Max[1] - bounds.Min[1]) / 2.0)
	box := vec2d.Rect{Min: vec2d.T{float64(x)*tileSize[0] - hw, float64(x+1)*tileSize[0] - hw}, Max: vec2d.T{float64(y+1)*tileSize[1] - hh, float64(y)*tileSize[1] - hh}}
	return NewTile(NewTileIndexFromLevelAndRowCol(uint32(z), uint32(y), uint32(x)), box)
}

type LocalStorage struct {
	Storage
	root string
}

func fileExist(path string) bool {
	_, err := os.Lstat(path)
	return !os.IsNotExist(err)
}

func walkDir(dirPth, suffix string) (files []string, err error) {
	files = make([]string, 0, 30)
	suffix = strings.ToUpper(suffix)

	err = filepath.Walk(dirPth, func(filename string, fi os.FileInfo, err error) error {
		if fi.IsDir() {
			return nil
		}

		if strings.HasSuffix(strings.ToUpper(fi.Name()), suffix) {
			files = append(files, filename)
		}
		return nil
	})

	return files, err
}

func (l *LocalStorage) WriteTile(tile *VoxelTile) bool {
	if !tile.vpixel.Vaild() {
		return false
	}
	path := MakeVoxelTileName(l.root, &tile.Tile, VOXEL_TILE_EXT)
	if l.TileExists(&tile.Tile) {
		path_tmp := MakeVoxelTileName(l.root, &tile.Tile, VOXEL_TILE_EXT_TEMP)
		err := tile.vpixel.Write(path_tmp)
		if err == nil {
			l.RemoveTile(&tile.Tile)
			return os.Rename(path_tmp, path) == nil
		}
	} else {
		err := tile.vpixel.Write(path)
		return err == nil
	}
	return false
}

func (l *LocalStorage) ReadTile(tile *VoxelTile) bool {
	if !tile.vpixel.Vaild() {
		return false
	}
	if l.TileExists(&tile.Tile) {
		path := MakeVoxelTileName(l.root, &tile.Tile, VOXEL_TILE_EXT)
		return tile.vpixel.Read(path) != nil
	}
	return false
}

func (l *LocalStorage) RemoveTile(tile *Tile) bool {
	if l.TileExists(tile) {
		curr_path := MakeVoxelTileName(l.root, tile, VOXEL_TILE_EXT)
		return os.Remove(curr_path) == nil
	}
	return false
}

func (l *LocalStorage) TileExists(tile *Tile) bool {
	path := MakeVoxelTileName(l.root, tile, VOXEL_TILE_EXT)
	return fileExist(path)
}

func (l *LocalStorage) ListTiles(bbox vec2d.Rect, tileSize vec2d.T) []Tile {
	files, err := walkDir(l.root, VOXEL_TILE_EXT)
	if err != nil {
		return nil
	}
	tils := make([]Tile, len(files))
	for i, v := range files {
		tils[i] = *ParseVoxelTileNameToTile(v, bbox, tileSize)
	}
	return tils
}

func (l *LocalStorage) UpdateTiles(tiles []*VoxelTile) bool {
	for _, v := range tiles {
		if v.IsDirty() {
			l.WriteTile(v)
		}
	}
	return true
}

func GetFileModTime(path string) int64 {
	f, err := os.Open(path)
	if err != nil {
		return -1
	}
	defer f.Close()

	fi, err := f.Stat()
	if err != nil {
		return -1
	}

	return fi.ModTime().Unix()
}

func (l *LocalStorage) TileModTime(tile *Tile) int64 {
	path := MakeVoxelTileName(l.root, tile, VOXEL_TILE_EXT)
	if fileExist(path) {
		return GetFileModTime(path)
	}
	return -1
}
