package vdb

// #include <stdlib.h>
// #include <stdint.h>
// #include "tile_index_api.h"
// #cgo CFLAGS: -I ./lib
// #cgo CXXFLAGS: -I ./lib
import "C"
import (
	"math"
	"unsafe"

	vec2d "github.com/flywave/go3d/float64/vec2"
)

type TileIndex struct {
	path uint64
}

const (
	QT_DEFAULT_MAX_LEVEL = uint32(24)
)

var (
	_order = [][]uint64{{0, 3}, {1, 2}}
)

var (
	MAX_LATITUDE = R2D * (2*math.Atan(math.Exp(180*D2R)) - M_PI_by2)
)

func Min(x, y uint32) uint32 {
	if x < y {
		return x
	}
	return y
}

func Max(x, y uint32) uint32 {
	if x > y {
		return x
	}
	return y
}

func NewTileIndex() *TileIndex {
	return &TileIndex{path: 0}
}

func NewTileIndexFromLevelAndRowCol(level uint32, row uint32, col uint32) *TileIndex {
	p := &TileIndex{path: 0}
	p.path = uint64(C.tile_index_new_from_level_row_col(C.uint(level), C.uint(row), C.uint(col)))
	return p
}

func NewTileIndexFromLevelAndBitList(level uint32, blist []byte) *TileIndex {
	p := &TileIndex{path: 0}
	p.from_branchlist(level, blist)
	return p
}

func NewTileIndexFromBitList(blist string) *TileIndex {
	p := &TileIndex{path: 0}
	p.from_branchlist(uint32(len(blist)), []byte(blist))
	return p
}

func NewTileIndexFromBitListAndLevel(other *TileIndex, level uint32) *TileIndex {
	p := &TileIndex{path: 0}
	p.path = uint64(C.tile_index_new_from_other(C.ulong(other.path), C.uint(level)))
	return p
}

func geodeticToTile(lon float64, lat float64, zoom uint8) *TileIndex {
	x := math.Floor((lon + 180.0) / 360.0 * math.Pow(2.0, float64(zoom)))
	y := math.Floor((1.0 + math.Log(math.Tan(lat*math.Pi/180.0)+
		1.0/math.Cos(lat*math.Pi/180.0))/
		math.Pi) /
		2.0 * math.Pow(2.0, float64(zoom)))
	return NewTileIndexFromLevelAndRowCol(uint32(zoom), uint32(y), uint32(x))
}

func lonlat2merc(x []float64, y []float64, pointCount int) bool {
	for i := 0; i < pointCount; i++ {
		if x[i] > 180 {
			x[i] = 180
		} else if x[i] < -180 {
			x[i] = -180
		}
		if y[i] > MAX_LATITUDE {
			y[i] = MAX_LATITUDE
		} else if y[i] < -MAX_LATITUDE {
			y[i] = -MAX_LATITUDE
		}
		x[i] = x[i] * MAXEXTENTby180
		y[i] = math.Log(math.Tan((90+y[i])*M_PIby360)) * R2D
		y[i] = y[i] * MAXEXTENTby180
	}
	return true
}

func merc2lonlat(x []float64, y []float64, pointCount int) bool {
	for i := 0; i < pointCount; i++ {
		if x[i] > MAXEXTENT {
			x[i] = MAXEXTENT
		} else if x[i] < -MAXEXTENT {
			x[i] = -MAXEXTENT
		}
		if y[i] > MAXEXTENT {
			y[i] = MAXEXTENT
		} else if y[i] < -MAXEXTENT {
			y[i] = -MAXEXTENT
		}
		x[i] = (x[i] / MAXEXTENT) * 180
		y[i] = (y[i] / MAXEXTENT) * 180
		y[i] = R2D * (2*math.Atan(math.Exp(y[i]*D2R)) - M_PI_by2)
	}
	return true
}

func getTileFromBox(minx float64, miny float64, maxx float64, maxy float64) *TileIndex {
	x := vec2d.T{minx, miny}
	y := vec2d.T{maxx, maxy}
	lonlat2merc(x[:], y[:], 2)
	_box := vec2d.Rect{Min: x, Max: y}
	_zoom := uint8(
		math.Floor(math.Log((2*math.Pi)/_box.Size())/math.Log(2)) - 1)

	if _box.Size() == 0 {
		_zoom = uint8(WEB_GLOBE_MAX_ZOOM)
	}

	if _zoom > uint8(WEB_GLOBE_MAX_ZOOM) {
		_zoom = uint8(WEB_GLOBE_MAX_ZOOM)
	}
	return geodeticToTile((minx+maxx)/2.0, (miny+maxy)/2.0, _zoom)
}

func NewTileIndexFromBox(box vec2d.Rect) *TileIndex {
	p := getTileFromBox(box.Min[0], box.Min[1], box.Max[0], box.Max[1])
	return p
}

func (q *TileIndex) Valid() bool {
	return bool(C.tile_index_path_is_valid(C.ulong(q.path)))
}

func (q *TileIndex) Less(other *TileIndex) bool {
	return bool(C.tile_index_less(C.ulong(q.path), C.ulong(other.path)))
}

func (q *TileIndex) Greater(other *TileIndex) bool {
	return other.Less(q)
}

func (q *TileIndex) Equal(other *TileIndex) bool {
	return other.path == q.path
}

func (q *TileIndex) Path() uint64 {
	return q.path
}

func (q *TileIndex) ToString() string {
	result := make([]byte, int(q.GetLevel()))

	for i := 0; i < int(q.GetLevel()); i++ {
		result[i] = byte('0') + byte(uint32(C.tile_index_level_bits_at_pos(C.ulong(q.path), C.uint(i))))
	}
	return string(result)
}

func (q *TileIndex) GetGenerationSequence() uint64 {
	return uint64(C.tile_index_new_get_generation_sequence(C.ulong(q.path)))
}

func (q *TileIndex) Parent() *TileIndex {
	path := uint64(C.tile_index_get_parent(C.ulong(q.path)))
	return &TileIndex{path: path}
}

func (q *TileIndex) Child(child uint32) *TileIndex {
	path := uint64(C.tile_index_get_child(C.ulong(q.path), C.uint(child)))
	return &TileIndex{path: path}
}

func (q *TileIndex) WhichChild() uint32 {
	return uint32(C.tile_index_which_child(C.ulong(q.path)))
}

func (q *TileIndex) AdvanceInLevel() bool {
	return bool(C.tile_index_advance_in_level((*C.ulong)(unsafe.Pointer(&q.path))))
}

func (q *TileIndex) Advance(max_level uint32) bool {
	return bool(C.tile_index_advance((*C.ulong)(unsafe.Pointer(&q.path)), C.uint(max_level)))
}

func (q *TileIndex) IsAncestorOf(other *TileIndex) bool {
	return bool(C.tile_index_is_ancestor_of(C.ulong(q.path), C.ulong(other.path)))
}

func IsPostOrder(path1 *TileIndex, path2 *TileIndex) bool {
	return !path1.IsAncestorOf(path2) &&
		(path2.IsAncestorOf(path1) || path2.Greater(path1))
}

func (q *TileIndex) GetLevelRowCol() (level uint32, row uint32, col uint32) {
	C.tile_index_get_level_row_col(C.ulong(q.path), (*C.uint)(unsafe.Pointer(&level)), (*C.uint)(unsafe.Pointer(&row)), (*C.uint)(unsafe.Pointer(&col)))
	return
}

func (q *TileIndex) GetLevel() uint32 {
	return uint32(C.tile_index_path_level(C.ulong(q.path)))
}

func (q *TileIndex) ChildTileCoordinates(tile_width uint32, child *TileIndex) (success bool, level uint32, row uint32, col uint32) {
	if !q.IsAncestorOf(child) {
		success = false
		return
	}

	relative_qpath := RelativePath(q, child)
	level = tile_width
	row = 0
	col = 0
	for level_ := 0; level_ < int(relative_qpath.GetLevel()) && level_ > 1; level_++ {
		quad := relative_qpath.Get(level)
		level >>= 1
		if quad == 0 {
			row += level
		} else if quad == 1 {
			row += level
			col += level
		} else if quad == 2 {
			col += level
		}
	}
	success = true
	return
}

func (q *TileIndex) Concatenate(sub_path *TileIndex) *TileIndex {
	path := uint64(C.tile_index_concatenate(C.ulong(q.path), C.ulong(sub_path.path)))
	return &TileIndex{path: path}
}

func (q *TileIndex) ToIndex(level uint32) uint64 {
	return uint64(C.tile_index_as_index(C.ulong(q.path), C.uint(level)))
}

func (q *TileIndex) Get(position uint32) uint32 {
	return uint32(C.tile_index_level_bits_at_pos(C.ulong(q.path), C.uint(position)))
}

func (q *TileIndex) from_branchlist(level uint32, blist []byte) {
	q.path = uint64(C.tile_index_new_from_branchlist(C.uint(level), (*C.uchar)(unsafe.Pointer(&blist[0]))))
}

func RelativePath(parent *TileIndex, child *TileIndex) *TileIndex {
	path := uint64(C.tile_index_relative_path(C.ulong(parent.path), C.ulong(child.path)))
	return &TileIndex{path: path}
}

func quadToBufferOffset(quad uint32, tileWidth uint32, tileHeight uint32) uint32 {
	switch quad {
	case 0:
		return 0
	case 1:
		return tileWidth / 2
	case 2:
		return (tileHeight * tileWidth) / 2
	case 3:
		return ((tileHeight + 1) * tileWidth) / 2
	}
	return 0
}

func magnifyQuadAddr(inRow uint32, inCol uint32, inQuad uint32) (outRow uint32, outCol uint32) {
	switch inQuad {
	case 0:
		outRow = inRow * 2
		outCol = inCol * 2
		break
	case 1:
		outRow = inRow * 2
		outCol = (inCol * 2) + 1
		break
	case 2:
		outRow = (inRow * 2) + 1
		outCol = inCol * 2
		break
	case 3:
		outRow = (inRow * 2) + 1
		outCol = (inCol * 2) + 1
		break
	}
	return
}
