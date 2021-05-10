package vdb

// #include <stdlib.h>
// #include <stdint.h>
// #cgo CFLAGS: -I ./lib
// #cgo CXXFLAGS: -I ./lib
// uint64_t fix_numeric_overflow(uint64_t path_mask, uint64_t level_bit) {
// return (path_mask << level_bit);
// }
import "C"
import (
	"math"
)

type QuadtreePath struct {
	path uint64
}

const (
	QT_LEVEL_BITS        = uint32(2)
	QT_LEVEL_BIT_MASK    = uint64(0x03)
	QT_TOTAL_BITS        = uint32(64)
	QT_DEFAULT_MAX_LEVEL = uint32(24)
	QT_PATH_MASK         = ^uint64(^uint64(0) >> (QT_DEFAULT_MAX_LEVEL * QT_LEVEL_BITS))
	QT_LEVEL_MASK        = ^QT_PATH_MASK
	WEB_GLOBE_MAX_ZOOM   = uint32(20)
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

func NewQuadtreePath() *QuadtreePath {
	return &QuadtreePath{path: 0}
}

func NewQuadtreePathFromLevelAndRowCol(level uint32, row uint32, col uint32) *QuadtreePath {
	p := &QuadtreePath{path: 0}
	for j := 0; j < int(level); j++ {
		right := int(0x01 & (col >> (level - uint32(j) - 1)))
		top := int(0x01 & (row >> (level - uint32(j) - 1)))
		p.path |= _order[right][top] << (QT_TOTAL_BITS - ((uint32(j) + 1) * QT_LEVEL_BITS))
	}

	p.path |= uint64(level)
	return p
}

func NewQuadtreePathFromLevelAndBitList(level uint32, blist []byte) *QuadtreePath {
	p := &QuadtreePath{path: 0}
	p.from_branchlist(level, blist)
	return p
}

func NewQuadtreePathFromBitList(blist string) *QuadtreePath {
	p := &QuadtreePath{path: 0}
	p.from_branchlist(uint32(len(blist)), []byte(blist))
	return p
}

func NewQuadtreePathFromBitListAndLevel(other *QuadtreePath, level uint32) *QuadtreePath {
	lev := Min(level, other.GetLevel())
	p := &QuadtreePath{path: 0}
	p.path = other.path_bits_level(lev) | uint64(lev)
	return p
}

func geodeticToTile(lon float64, lat float64, zoom uint8) *QuadtreePath {
	x := math.Floor((lon + 180.0) / 360.0 * math.Pow(2.0, float64(zoom)))
	y := math.Floor((1.0 + math.Log(math.Tan(lat*math.Pi/180.0)+
		1.0/math.Cos(lat*math.Pi/180.0))/
		math.Pi) /
		2.0 * math.Pow(2.0, float64(zoom)))
	return NewQuadtreePathFromLevelAndRowCol(uint32(zoom), uint32(y), uint32(x))
}

const (
	EARTH_RADIUS        = float64(6378137.0)
	EARTH_DIAMETER      = EARTH_RADIUS * 2.0
	EARTH_CIRCUMFERENCE = EARTH_DIAMETER * math.Pi
	MAXEXTENT           = EARTH_CIRCUMFERENCE / 2.0
	M_PI_by2            = float64(math.Pi / 2)
	D2R                 = float64(math.Pi / 180)
	R2D                 = float64(180 / math.Pi)
	M_PIby360           = float64(math.Pi / 360)
	MAXEXTENTby180      = float64(MAXEXTENT / 180)
)

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

func getTileFromBox(minx float64, miny float64, maxx float64, maxy float64) *QuadtreePath {
	x := [2]float64{minx, miny}
	y := [2]float64{maxx, maxy}
	lonlat2merc(x[:], y[:], 2)
	_box := BBox2d{x[0], y[0], x[1], y[1]}
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

func NewQuadtreePathFromBox(box BBox2d) *QuadtreePath {
	p := getTileFromBox(box[0], box[1], box[2], box[3])
	return p
}

func (q *QuadtreePath) Valid() bool {
	return q.GetLevel() <= QT_DEFAULT_MAX_LEVEL &&
		(0 == (q.path & ^(q.path_mask(q.GetLevel()) | QT_LEVEL_MASK)))
}

func (q *QuadtreePath) Less(other *QuadtreePath) bool {
	minlev := other.GetLevel()
	if q.GetLevel() < other.GetLevel() {
		minlev = q.GetLevel()
	}
	mask := ^(^uint64(0) >> (minlev * QT_LEVEL_BITS))
	if (mask & (q.path ^ other.path)) > 0 {
		return q.path_bits() < other.path_bits()
	} else {
		return q.GetLevel() < other.GetLevel()
	}
}

func (q *QuadtreePath) Greater(other *QuadtreePath) bool {
	return other.Less(q)
}

func (q *QuadtreePath) Equal(other *QuadtreePath) bool {
	return other.path == q.path
}

func (q *QuadtreePath) Path() uint64 {
	return q.path
}

func (q *QuadtreePath) ToString() string {
	result := make([]byte, int(q.GetLevel()))

	for i := 0; i < int(q.GetLevel()); i++ {
		result[i] = byte('0') + byte(q.level_bits_at_pos(uint32(i)))
	}
	return string(result)
}

func (q *QuadtreePath) GetGenerationSequence() uint64 {
	level_ := q.GetLevel()
	sequence := q.path
	check_for_2_or_3_mask := (uint64(0x1)) << (QT_TOTAL_BITS - 1)
	interchange_2_or_3_mask := (uint64(0x01)) << (QT_TOTAL_BITS - 2)

	for j := 0; j < int(level_); j++ {
		if (sequence & check_for_2_or_3_mask) > 0 {
			sequence ^= interchange_2_or_3_mask
		}
		check_for_2_or_3_mask >>= 2
		interchange_2_or_3_mask >>= 2
	}
	return sequence
}

func (q *QuadtreePath) Parent() *QuadtreePath {
	new_level := q.GetLevel() - 1

	return &QuadtreePath{path: (q.path & (uint64(C.fix_numeric_overflow(C.ulong(QT_PATH_MASK), C.ulong(QT_LEVEL_BITS))) * uint64(QT_DEFAULT_MAX_LEVEL-new_level))) | uint64(new_level)}
}

func (q *QuadtreePath) Child(child uint32) *QuadtreePath {
	new_level := q.GetLevel() + 1
	return &QuadtreePath{path: q.path_bits() | uint64(child)<<(QT_TOTAL_BITS-new_level*QT_LEVEL_BITS) | uint64(new_level)}
}

func (q *QuadtreePath) WhichChild() uint32 {
	return uint32((q.path >> (QT_TOTAL_BITS - q.GetLevel()*QT_LEVEL_BITS)) & QT_LEVEL_BIT_MASK)
}

func (q *QuadtreePath) AdvanceInLevel() bool {
	path_bits_ := q.path_bits()
	path_mask_ := q.path_mask(q.GetLevel())
	if path_bits_ != path_mask_ {
		q.path += uint64(1) << (QT_TOTAL_BITS - q.GetLevel()*QT_LEVEL_BITS)
		return true
	} else {
		return false
	}
}

func (q *QuadtreePath) Advance(max_level uint32) bool {
	if q.GetLevel() < max_level {
		q.path = q.Child(0).path
		return true
	} else {
		for q.WhichChild() == 4-1 {
			q.path = q.Parent().path
		}
		return q.AdvanceInLevel()
	}
}

func (q *QuadtreePath) IsAncestorOf(other *QuadtreePath) bool {
	if q.GetLevel() <= other.GetLevel() {
		return q.path_bits_level(q.GetLevel()) == other.path_bits_level(q.GetLevel())
	}
	return false
}

func IsPostOrder(path1 *QuadtreePath, path2 *QuadtreePath) bool {
	return !path1.IsAncestorOf(path2) &&
		(path2.IsAncestorOf(path1) || path2.Greater(path1))
}

func (q *QuadtreePath) GetLevelRowCol() (level uint32, row uint32, col uint32) {
	rowbits := []uint32{0x00, 0x00, 0x01, 0x01}
	colbits := []uint32{0x00, 0x01, 0x01, 0x00}

	row_val := uint32(0)
	col_val := uint32(0)

	for j := 0; j < int(q.GetLevel()); j++ {
		level_bits := q.level_bits_at_pos(uint32(j))
		row_val = (row_val << 1) | (rowbits[level_bits])
		col_val = (col_val << 1) | (colbits[level_bits])
	}

	level = q.GetLevel()
	row = row_val
	col = col_val
	return
}

func (q *QuadtreePath) GetLevel() uint32 {
	return uint32(q.path & QT_LEVEL_MASK)
}

func (q *QuadtreePath) ChildTileCoordinates(tile_width uint32, child *QuadtreePath) (success bool, level uint32, row uint32, col uint32) {
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

func (q *QuadtreePath) Concatenate(sub_path *QuadtreePath) *QuadtreePath {
	level_ := q.GetLevel() + sub_path.GetLevel()
	return &QuadtreePath{path: (q.path & QT_PATH_MASK) |
		((sub_path.path & QT_PATH_MASK) >> q.GetLevel() * uint64(QT_LEVEL_BITS)) | uint64(level_)}
}

func (q *QuadtreePath) ToIndex(level uint32) uint64 {
	return (q.path >> (QT_TOTAL_BITS - level*QT_LEVEL_BITS))
}

func (q *QuadtreePath) Get(position uint32) uint32 {
	return q.level_bits_at_pos(position)
}

func (q *QuadtreePath) path_bits() uint64 { return q.path & QT_PATH_MASK }

func (q *QuadtreePath) path_mask(level uint32) uint64 {
	return QT_PATH_MASK << ((QT_DEFAULT_MAX_LEVEL - level) * QT_LEVEL_BITS)
}

func (q *QuadtreePath) path_bits_level(level uint32) uint64 {
	return q.path & q.path_mask(level)
}

func (q *QuadtreePath) level_bits_at_pos(position uint32) uint32 {
	return uint32((q.path >> (QT_TOTAL_BITS - (position+1)*QT_LEVEL_BITS)) &
		QT_LEVEL_BIT_MASK)
}

func (q *QuadtreePath) from_branchlist(level uint32, blist []byte) {
	for j := 0; j < int(level); j++ {
		q.path |= (uint64(blist[j]) & QT_LEVEL_BIT_MASK) << (QT_TOTAL_BITS - (uint32(j+1) * QT_LEVEL_BITS))
	}
	q.path |= uint64(level)
}

func RelativePath(parent *QuadtreePath, child *QuadtreePath) *QuadtreePath {
	levelDiff := child.GetLevel() - parent.GetLevel()
	return &QuadtreePath{path: (child.path_bits() << (parent.GetLevel() * QT_LEVEL_BITS)) |
		uint64(levelDiff)}
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
